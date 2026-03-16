# WIA-FUSION Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the RESTful API interfaces for nuclear fusion energy systems, enabling programmatic access to plasma data, control systems, and analytics.

### 1.1 Base URL

```
Production: https://api.fusion.wia.live/v1
Staging: https://api-staging.fusion.wia.live/v1
```

### 1.2 Authentication

```yaml
type: Bearer Token (JWT)
header: Authorization: Bearer <token>
expiry: 24 hours
refresh: /auth/refresh
```

---

## 2. API Endpoints

### 2.1 Plasma State API

#### POST /plasma/state
Record plasma state data.

```yaml
Request:
  Content-Type: application/json
  Body:
    plasma_state:
      shot_id: string (required)
      timestamp: ISO8601 (required)
      reactor: enum[ITER|KSTAR|JET|SPARC|custom]
      core_parameters:
        temperature_keV: { ion: number, electron: number }
        density_m3: { value: number, unit: "1e20/m3" }
        confinement_time_s: number
      performance:
        q_factor: number
        fusion_power_mw: number

Response: 201 Created
  {
    "success": true,
    "shot_id": "KSTAR-20240415-001",
    "recorded_at": "2024-04-15T10:30:00Z"
  }
```

#### GET /plasma/state/{shot_id}
Retrieve plasma state for a specific shot.

```yaml
Parameters:
  shot_id: string (path, required)
  fields: string (query, optional) - Comma-separated field list

Response: 200 OK
  {
    "plasma_state": { ... }
  }
```

#### GET /plasma/state/latest
Get the most recent plasma state.

```yaml
Parameters:
  reactor: string (query, optional)

Response: 200 OK
  {
    "plasma_state": { ... },
    "age_ms": 150
  }
```

---

### 2.2 Stability Analysis API

#### GET /plasma/stability/{shot_id}
Analyze plasma stability.

```yaml
Parameters:
  shot_id: string (path, required)
  time_window_s: number (query, optional, default: 1.0)

Response: 200 OK
  {
    "stability_analysis": {
      "disruption_risk": 0.15,
      "confidence": 0.92,
      "mhd_modes": ["m=2,n=1", "m=3,n=2"],
      "elm_type": "Type-I",
      "elm_frequency_hz": 50,
      "recommendations": [
        "Maintain current heating profile",
        "Monitor m=2,n=1 mode amplitude"
      ]
    }
  }
```

---

### 2.3 AI Control Optimization API

#### POST /plasma/control/optimize
Request AI-optimized control parameters.

```yaml
Request:
  {
    "current_state": {
      "temperature_keV": 10,
      "density_m3": 1.0,
      "confinement_time_s": 2.5,
      "heating_power_mw": { "nbi": 30, "icrh": 10, "ecrh": 5 }
    },
    "target": {
      "q_factor": 10,
      "steady_state_duration_s": 300
    },
    "constraints": {
      "max_heating_power_mw": 50,
      "max_divertor_heat_mw_m2": 10
    }
  }

Response: 200 OK
  {
    "optimized_control": {
      "heating_power_mw": {
        "nbi": 33,
        "icrh": 10,
        "ecrh": 7
      },
      "plasma_shape": {
        "elongation": 1.85,
        "triangularity": 0.5
      },
      "predicted_performance": {
        "q_factor": 10.2,
        "confinement_improvement": "+15%"
      },
      "confidence": 0.88
    }
  }
```

---

### 2.4 Energy Balance API

#### GET /fusion/energy-balance/{shot_id}
Analyze energy balance for a shot.

```yaml
Parameters:
  shot_id: string (path, required)

Response: 200 OK
  {
    "energy_balance": {
      "input_power_mw": {
        "ohmic": 1,
        "nbi": 33,
        "icrh": 10,
        "ecrh": 6,
        "total": 50
      },
      "output_power_mw": {
        "fusion": 500,
        "radiation": 50,
        "conduction": 30,
        "convection": 20
      },
      "q_factor": 10.0,
      "energy_confinement_time_s": 3.2,
      "h_factor": 1.0
    }
  }
```

---

### 2.5 Disruption Prediction API

#### POST /fusion/predict/disruption
AI-powered disruption prediction.

```yaml
Request:
  {
    "plasma_state": {
      "temperature_keV": 10,
      "density_m3": 1.0,
      "plasma_current_ma": 15,
      "beta_percent": 2.5,
      "li": 0.85,
      "q95": 3.0
    },
    "diagnostics": {
      "mhd_amplitude": 0.3,
      "locked_mode_indicator": false,
      "radiation_peaking": 1.2
    }
  }

Response: 200 OK
  {
    "prediction": {
      "disruption_probability": 0.15,
      "time_to_disruption_s": null,
      "confidence": 0.92,
      "risk_factors": [
        { "factor": "mhd_activity", "contribution": 0.08 },
        { "factor": "density_limit", "contribution": 0.05 },
        { "factor": "beta_limit", "contribution": 0.02 }
      ],
      "recommended_actions": [
        { "action": "reduce_density", "priority": "low" },
        { "action": "monitor_mhd", "priority": "medium" }
      ]
    }
  }
```

---

### 2.6 Shot Database API

#### GET /shots
List plasma shots with filtering.

```yaml
Parameters:
  reactor: string (query, optional)
  date_from: ISO8601 (query, optional)
  date_to: ISO8601 (query, optional)
  min_q_factor: number (query, optional)
  limit: integer (query, default: 100, max: 1000)
  offset: integer (query, default: 0)

Response: 200 OK
  {
    "shots": [
      {
        "shot_id": "KSTAR-20240415-001",
        "timestamp": "2024-04-15T10:30:00Z",
        "reactor": "KSTAR",
        "duration_s": 48,
        "q_factor": 1.2,
        "max_temperature_keV": 10
      }
    ],
    "total": 1500,
    "limit": 100,
    "offset": 0
  }
```

---

## 3. WebSocket Streaming API

### 3.1 Real-time Plasma Stream

```yaml
Endpoint: wss://api.fusion.wia.live/v1/stream/plasma
Protocol: WebSocket
Authentication: Query parameter ?token=<jwt>
```

#### Subscribe Message
```json
{
  "action": "subscribe",
  "channels": ["plasma_state", "stability", "control"],
  "reactor": "KSTAR",
  "sample_rate_hz": 100
}
```

#### Data Message
```json
{
  "channel": "plasma_state",
  "timestamp": "2024-04-15T10:30:00.150Z",
  "data": {
    "temperature_keV": 10.2,
    "density_m3": 1.05,
    "q_factor": 1.18
  }
}
```

---

## 4. Error Handling

### 4.1 Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid plasma state data",
    "details": [
      { "field": "temperature_keV", "error": "must be positive" }
    ],
    "request_id": "req-12345"
  }
}
```

### 4.2 Error Codes

| HTTP Status | Code | Description |
|-------------|------|-------------|
| 400 | VALIDATION_ERROR | Invalid request data |
| 401 | UNAUTHORIZED | Missing or invalid token |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | NOT_FOUND | Resource not found |
| 429 | RATE_LIMITED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | Maintenance mode |

---

## 5. Rate Limiting

```yaml
Default Limits:
  - Anonymous: 100 requests/hour
  - Authenticated: 10,000 requests/hour
  - Premium: 100,000 requests/hour

Headers:
  X-RateLimit-Limit: 10000
  X-RateLimit-Remaining: 9950
  X-RateLimit-Reset: 1713177600
```

---

## 6. SDK Examples

### 6.1 TypeScript/JavaScript

```typescript
import { FusionClient } from '@wia/fusion-sdk';

const client = new FusionClient({
  apiKey: process.env.WIA_FUSION_API_KEY,
  reactor: 'KSTAR'
});

// Record plasma state
await client.plasma.recordState({
  temperature_keV: { ion: 10, electron: 10 },
  density_m3: 1.0,
  confinement_time_s: 3.0
});

// Get disruption prediction
const prediction = await client.fusion.predictDisruption({
  beta_percent: 2.5,
  mhd_amplitude: 0.3
});

console.log(`Disruption risk: ${prediction.probability * 100}%`);
```

### 6.2 Python

```python
from wia_fusion import FusionClient

client = FusionClient(api_key=os.environ['WIA_FUSION_API_KEY'])

# Stream real-time data
async for state in client.stream_plasma('KSTAR'):
    print(f"Q-factor: {state.q_factor}")
    if state.disruption_risk > 0.5:
        await client.control.emergency_shutdown()
```

---

## 7. Versioning

API versions follow semantic versioning:
- `/v1/` - Current stable version
- `/v2-beta/` - Next version preview

Deprecation notices are provided 6 months in advance.

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association
