# Chapter 4: API Interface Specification (Phase 2)

## 4.1 RESTful API Design Principles

### 4.1.1 API Architecture Overview

Phase 2 of WIA-BEMS defines the RESTful API interface that enables applications to access building energy data and control building systems. The API follows industry best practices and modern web standards to ensure wide compatibility and ease of integration.

**API Design Principles:**

```typescript
interface APIDesignPrinciples {
  restful: {
    resourceOriented: "Resources identified by URIs";
    stateless: "No server-side session state";
    uniformInterface: "Standard HTTP methods";
    hypermedia: "HATEOAS-inspired linking";
  };

  versioning: {
    strategy: "URI path versioning";
    format: "/api/v1/..., /api/v2/...";
    compatibility: "Semantic versioning";
    deprecation: "Minimum 12-month notice";
  };

  consistency: {
    naming: "lowercase-kebab-case for paths";
    pagination: "Cursor-based pagination";
    filtering: "Query parameter filtering";
    sorting: "Sort parameter with field:direction";
  };

  performance: {
    compression: "gzip/brotli supported";
    caching: "ETags and Cache-Control headers";
    pagination: "Default 100, max 1000 per page";
  };
}
```

### 4.1.2 Base URL and Versioning

**API Base Structure:**

```
Production:  https://api.{domain}/bems/v1
Staging:     https://api-staging.{domain}/bems/v1
Development: https://api-dev.{domain}/bems/v1

Version Format: /bems/v{major}
Current Version: v1

Example Endpoints:
├── https://api.example.com/bems/v1/buildings
├── https://api.example.com/bems/v1/buildings/{id}/energy
├── https://api.example.com/bems/v1/equipment/{id}/status
└── https://api.example.com/bems/v1/commands
```

## 4.2 Authentication and Authorization

### 4.2.1 OAuth 2.0 Implementation

WIA-BEMS uses OAuth 2.0 for authentication with support for multiple grant types depending on the use case:

**Supported Grant Types:**

| Grant Type | Use Case | Token Lifetime |
|------------|----------|----------------|
| Authorization Code + PKCE | User-facing applications | 1 hour access, 30 day refresh |
| Client Credentials | Machine-to-machine | 1 hour, no refresh |
| Device Authorization | IoT devices | 1 hour access, 90 day refresh |
| Refresh Token | Token renewal | 30-90 days |

**OAuth 2.0 Flow Example:**

```typescript
// Authorization Code Flow with PKCE

// Step 1: Generate PKCE parameters
function generatePKCE(): { verifier: string; challenge: string } {
  const verifier = base64url(crypto.randomBytes(32));
  const challenge = base64url(
    crypto.createHash('sha256').update(verifier).digest()
  );
  return { verifier, challenge };
}

// Step 2: Redirect to authorization endpoint
const authUrl = new URL('https://auth.example.com/oauth/authorize');
authUrl.searchParams.set('client_id', CLIENT_ID);
authUrl.searchParams.set('redirect_uri', REDIRECT_URI);
authUrl.searchParams.set('response_type', 'code');
authUrl.searchParams.set('scope', 'buildings:read energy:read equipment:control');
authUrl.searchParams.set('state', generateState());
authUrl.searchParams.set('code_challenge', pkce.challenge);
authUrl.searchParams.set('code_challenge_method', 'S256');

// Step 3: Exchange code for tokens
async function exchangeCode(code: string, verifier: string): Promise<TokenResponse> {
  const response = await fetch('https://auth.example.com/oauth/token', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: new URLSearchParams({
      grant_type: 'authorization_code',
      client_id: CLIENT_ID,
      code: code,
      redirect_uri: REDIRECT_URI,
      code_verifier: verifier
    })
  });

  return response.json();
}

// Token Response
interface TokenResponse {
  access_token: string;
  token_type: 'Bearer';
  expires_in: number;
  refresh_token: string;
  scope: string;
}
```

### 4.2.2 API Scopes

**Scope Hierarchy:**

```yaml
buildings:
  - buildings:read       # Read building information
  - buildings:write      # Create/update buildings
  - buildings:admin      # Full building management

energy:
  - energy:read          # Read energy data
  - energy:write         # Submit energy data
  - energy:export        # Export bulk data

environmental:
  - environmental:read   # Read temperature, IAQ
  - environmental:write  # Submit environmental data

occupancy:
  - occupancy:read       # Read occupancy data
  - occupancy:write      # Submit occupancy data

equipment:
  - equipment:read       # Read equipment status
  - equipment:control    # Send control commands
  - equipment:configure  # Configure equipment

schedules:
  - schedules:read       # Read schedules
  - schedules:write      # Create/modify schedules

alarms:
  - alarms:read          # View alarms
  - alarms:acknowledge   # Acknowledge alarms
  - alarms:configure     # Configure alarm rules

admin:
  - admin:users          # User management
  - admin:audit          # Audit log access
  - admin:settings       # System settings
```

### 4.2.3 Role-Based Access Control

**Standard Roles:**

| Role | Description | Typical Scopes |
|------|-------------|----------------|
| viewer | Read-only access | *:read |
| operator | Operational control | *:read, equipment:control, schedules:write |
| engineer | System configuration | All except admin:* |
| manager | Full building access | All except admin:settings |
| administrator | Full system access | All scopes |

**RBAC Implementation:**

```json
{
  "role": "operator",
  "permissions": [
    {
      "resource": "buildings",
      "actions": ["read"],
      "constraints": {
        "building_ids": ["BLDG-HQ001", "BLDG-HQ002"]
      }
    },
    {
      "resource": "equipment",
      "actions": ["read", "control"],
      "constraints": {
        "equipment_types": ["ahu", "vav", "lighting"]
      }
    },
    {
      "resource": "schedules",
      "actions": ["read", "write"],
      "constraints": null
    }
  ]
}
```

## 4.3 API Endpoints

### 4.3.1 Buildings API

**Building Endpoints:**

```yaml
# List Buildings
GET /buildings
  Query Parameters:
    - limit: integer (default 100, max 1000)
    - cursor: string (pagination cursor)
    - type: string (filter by building type)
    - search: string (search by name)
  Response: 200 OK
    {
      "buildings": [...],
      "pagination": {
        "next_cursor": "abc123",
        "has_more": true
      }
    }

# Get Building
GET /buildings/{building_id}
  Response: 200 OK
    {
      "building_id": "BLDG-HQ001",
      "name": "Headquarters Building",
      "type": "office",
      "area": {...},
      "_links": {
        "self": "/buildings/BLDG-HQ001",
        "energy": "/buildings/BLDG-HQ001/energy",
        "equipment": "/buildings/BLDG-HQ001/equipment"
      }
    }

# Create Building
POST /buildings
  Request Body: Building object
  Response: 201 Created

# Update Building
PUT /buildings/{building_id}
  Request Body: Building object
  Response: 200 OK

# Delete Building
DELETE /buildings/{building_id}
  Response: 204 No Content
```

### 4.3.2 Energy Data API

**Energy Endpoints:**

```yaml
# Query Energy Consumption
GET /buildings/{building_id}/energy
  Query Parameters:
    - start: ISO 8601 datetime (required)
    - end: ISO 8601 datetime (required)
    - interval: 15m | 1h | 1d | 1M (default 1h)
    - energy_type: electricity | gas | all
    - meter_id: filter by meter
    - include_cost: boolean
    - include_carbon: boolean
  Response: 200 OK
    {
      "building_id": "BLDG-HQ001",
      "period": {
        "start": "2025-01-01T00:00:00Z",
        "end": "2025-01-31T23:59:59Z"
      },
      "interval": "1h",
      "data": [
        {
          "timestamp": "2025-01-01T00:00:00Z",
          "energy_kwh": 245.5,
          "power_kw": 245.5,
          "cost_usd": 24.55,
          "carbon_kg": 98.2
        },
        ...
      ],
      "summary": {
        "total_energy_kwh": 180500,
        "peak_demand_kw": 450,
        "average_power_kw": 242,
        "total_cost_usd": 18050,
        "total_carbon_kg": 72200
      }
    }

# Get Real-time Power
GET /buildings/{building_id}/power
  Response: 200 OK
    {
      "timestamp": "2025-01-15T14:30:00Z",
      "power_kw": 385.2,
      "power_factor": 0.92,
      "voltage_v": 480.5,
      "current_a": 463.2
    }

# Submit Energy Data
POST /buildings/{building_id}/energy
  Request Body: EnergyConsumption object
  Response: 201 Created

# Bulk Energy Upload
POST /buildings/{building_id}/energy/bulk
  Request Body: Array of EnergyConsumption
  Response: 202 Accepted
    {
      "job_id": "JOB-123",
      "status": "processing",
      "records_submitted": 744
    }

# Check Bulk Upload Status
GET /jobs/{job_id}
  Response: 200 OK
    {
      "job_id": "JOB-123",
      "status": "completed",
      "records_processed": 744,
      "records_failed": 2,
      "errors": [...]
    }
```

### 4.3.3 Equipment API

**Equipment Management Endpoints:**

```yaml
# List Equipment
GET /buildings/{building_id}/equipment
  Query Parameters:
    - type: ahu | vav | chiller | ...
    - status: running | stopped | alarm
    - location_id: filter by location
  Response: 200 OK

# Get Equipment Details
GET /equipment/{equipment_id}
  Response: 200 OK
    {
      "equipment_id": "AHU-301",
      "type": "ahu",
      "name": "AHU 3rd Floor",
      "location": {...},
      "status": {...},
      "setpoints": {...},
      "feedback": {...}
    }

# Get Equipment History
GET /equipment/{equipment_id}/history
  Query Parameters:
    - start: ISO 8601
    - end: ISO 8601
    - points: comma-separated list of points
  Response: 200 OK

# Send Command
POST /equipment/{equipment_id}/commands
  Request Body:
    {
      "command": "set_temperature",
      "parameters": {
        "supply_air_temp_setpoint_c": 14.0
      },
      "priority": 8,
      "duration_minutes": 60
    }
  Response: 202 Accepted
    {
      "command_id": "CMD-456",
      "status": "pending",
      "expires_at": "2025-01-15T15:30:00Z"
    }

# Get Command Status
GET /commands/{command_id}
  Response: 200 OK
    {
      "command_id": "CMD-456",
      "status": "executed",
      "executed_at": "2025-01-15T14:30:05Z",
      "result": "success"
    }
```

### 4.3.4 Environmental Data API

**Environmental Endpoints:**

```yaml
# Get Current Conditions
GET /buildings/{building_id}/environment
  Query Parameters:
    - location_id: specific location
  Response: 200 OK
    {
      "locations": [
        {
          "location_id": "LOC-3-OPEN",
          "name": "3rd Floor Open Office",
          "temperature_c": 22.5,
          "humidity_pct": 45,
          "co2_ppm": 620,
          "air_quality_index": 42,
          "timestamp": "2025-01-15T14:30:00Z"
        }
      ]
    }

# Get Environmental History
GET /locations/{location_id}/environment/history
  Query Parameters:
    - start: ISO 8601
    - end: ISO 8601
    - parameters: temperature,humidity,co2
  Response: 200 OK

# Set Environmental Setpoints
PUT /locations/{location_id}/setpoints
  Request Body:
    {
      "temperature_setpoint_c": 22.0,
      "humidity_setpoint_pct": 50,
      "co2_setpoint_ppm": 800
    }
  Response: 200 OK
```

## 4.4 Real-Time Data Streaming

### 4.4.1 WebSocket Interface

WIA-BEMS supports WebSocket connections for real-time data streaming:

**WebSocket Connection:**

```typescript
// Connect to WebSocket
const ws = new WebSocket('wss://api.example.com/bems/v1/stream');

// Authentication
ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    token: accessToken
  }));
};

// Subscribe to streams
ws.send(JSON.stringify({
  type: 'subscribe',
  streams: [
    {
      type: 'power_demand',
      building_id: 'BLDG-HQ001',
      meter_id: 'METER-E-301',
      sample_rate: '1s'
    },
    {
      type: 'equipment_status',
      equipment_ids: ['AHU-301', 'AHU-302'],
      sample_rate: '5s'
    },
    {
      type: 'environment',
      location_ids: ['LOC-3-OPEN'],
      parameters: ['temperature', 'co2'],
      sample_rate: '30s'
    }
  ]
}));

// Handle messages
ws.onmessage = (event) => {
  const message = JSON.parse(event.data);

  switch (message.type) {
    case 'power_demand':
      console.log(`Power: ${message.data.power_kw} kW`);
      break;
    case 'equipment_status':
      console.log(`${message.data.equipment_id}: ${message.data.status}`);
      break;
    case 'environment':
      console.log(`Temp: ${message.data.temperature_c}°C`);
      break;
    case 'alarm':
      console.log(`ALARM: ${message.data.description}`);
      break;
  }
};

// Unsubscribe
ws.send(JSON.stringify({
  type: 'unsubscribe',
  stream_ids: ['stream-123']
}));
```

### 4.4.2 Server-Sent Events

For simpler real-time needs, SSE is also supported:

```typescript
// SSE Connection
const eventSource = new EventSource(
  'https://api.example.com/bems/v1/buildings/BLDG-HQ001/events?token=' + accessToken
);

eventSource.addEventListener('power', (event) => {
  const data = JSON.parse(event.data);
  updatePowerDisplay(data.power_kw);
});

eventSource.addEventListener('alarm', (event) => {
  const data = JSON.parse(event.data);
  showAlarmNotification(data);
});

eventSource.onerror = (error) => {
  console.error('SSE error:', error);
  // Implement reconnection logic
};
```

## 4.5 Error Handling

### 4.5.1 HTTP Status Codes

**Standard Response Codes:**

| Code | Meaning | When Used |
|------|---------|-----------|
| 200 | OK | Successful GET, PUT |
| 201 | Created | Successful POST creating resource |
| 202 | Accepted | Async operation queued |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid request body/parameters |
| 401 | Unauthorized | Missing/invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Resource state conflict |
| 422 | Unprocessable | Validation errors |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Error | Server error |
| 503 | Service Unavailable | Maintenance/overload |

### 4.5.2 Error Response Format

**Standard Error Response:**

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Request validation failed",
    "details": [
      {
        "field": "energy.value",
        "code": "INVALID_RANGE",
        "message": "Energy value must be non-negative"
      },
      {
        "field": "timestamp_end",
        "code": "INVALID_ORDER",
        "message": "End timestamp must be after start timestamp"
      }
    ],
    "request_id": "req-abc123",
    "timestamp": "2025-01-15T14:30:00Z",
    "documentation_url": "https://docs.wia.org/bems/errors/VALIDATION_ERROR"
  }
}
```

**Error Codes:**

| Code | Description |
|------|-------------|
| AUTHENTICATION_REQUIRED | No authentication token provided |
| INVALID_TOKEN | Token expired or invalid |
| INSUFFICIENT_SCOPE | Token lacks required scope |
| RESOURCE_NOT_FOUND | Requested resource doesn't exist |
| VALIDATION_ERROR | Request body validation failed |
| RATE_LIMIT_EXCEEDED | Too many requests |
| CONFLICT | Resource state conflict |
| INTERNAL_ERROR | Unexpected server error |

## 4.6 Rate Limiting and Quotas

### 4.6.1 Rate Limit Headers

**Rate Limit Response Headers:**

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 850
X-RateLimit-Reset: 1705329600
X-RateLimit-Policy: 1000;w=3600
```

### 4.6.2 Rate Limit Tiers

| Tier | Rate (requests/hour) | Burst | Use Case |
|------|---------------------|-------|----------|
| Free | 100 | 10 | Development/testing |
| Basic | 1,000 | 50 | Single building |
| Professional | 10,000 | 200 | Multiple buildings |
| Enterprise | 100,000 | 1,000 | Portfolio management |
| Unlimited | ∞ | 5,000 | On-premise deployment |

### 4.6.3 Handling Rate Limits

```typescript
async function apiRequest(url: string, options: RequestInit): Promise<Response> {
  const response = await fetch(url, options);

  if (response.status === 429) {
    const retryAfter = parseInt(response.headers.get('Retry-After') || '60');

    console.log(`Rate limited. Retrying in ${retryAfter}s`);

    await sleep(retryAfter * 1000);
    return apiRequest(url, options);
  }

  return response;
}
```

## 4.7 SDK Implementation

### 4.7.1 TypeScript SDK

**Installation and Usage:**

```bash
npm install wia-bems-sdk
```

```typescript
import { WIABEMSClient, Building, EnergyData } from 'wia-bems-sdk';

// Initialize with OAuth token
const client = new WIABEMSClient({
  baseUrl: 'https://api.example.com/bems/v1',
  auth: {
    type: 'oauth',
    accessToken: 'your-access-token',
    refreshToken: 'your-refresh-token',
    onTokenRefresh: (newToken) => saveToken(newToken)
  },
  timeout: 30000,
  retries: 3
});

// Buildings API
const buildings: Building[] = await client.buildings.list({ type: 'office' });
const building: Building = await client.buildings.get('BLDG-HQ001');

// Energy API
const energy: EnergyData = await client.energy.query('BLDG-HQ001', {
  start: '2025-01-01T00:00:00Z',
  end: '2025-01-31T23:59:59Z',
  interval: '1h'
});

// Real-time streaming
const stream = client.realtime.connect('BLDG-HQ001');

stream.subscribePower('METER-E-301', { sampleRate: '1s' }, (data) => {
  console.log(`Power: ${data.power_kw} kW`);
});

stream.subscribeAlarms((alarm) => {
  console.log(`Alarm: ${alarm.description}`);
});

// Equipment control
const command = await client.equipment.sendCommand('AHU-301', {
  command: 'set_temperature',
  parameters: { supply_air_temp_setpoint_c: 13.5 },
  priority: 8,
  duration: 60
});

console.log(`Command ${command.command_id}: ${command.status}`);

// Cleanup
stream.disconnect();
```

### 4.7.2 Python SDK

```python
from wia_bems import WIABEMSClient, OAuthConfig
from datetime import datetime, timedelta

# Initialize client
client = WIABEMSClient(
    base_url='https://api.example.com/bems/v1',
    auth=OAuthConfig(
        access_token='your-access-token',
        refresh_token='your-refresh-token'
    )
)

# Query energy data
energy_data = client.energy.query(
    building_id='BLDG-HQ001',
    start=datetime.now() - timedelta(days=30),
    end=datetime.now(),
    interval='1h'
)

print(f"Total energy: {energy_data.summary.total_energy_kwh} kWh")

# Real-time streaming with async
import asyncio

async def handle_power(data):
    print(f"Power: {data.power_kw} kW")

async def main():
    async with client.realtime.connect('BLDG-HQ001') as stream:
        await stream.subscribe_power('METER-E-301', callback=handle_power)
        await asyncio.sleep(3600)  # Stream for 1 hour

asyncio.run(main())
```

## 4.8 API Testing and Validation

### 4.8.1 OpenAPI Specification

The complete API is documented using OpenAPI 3.1:

```yaml
openapi: 3.1.0
info:
  title: WIA-BEMS API
  version: 1.0.0
  description: Building Energy Management System API

servers:
  - url: https://api.example.com/bems/v1
    description: Production

security:
  - oauth2: []

paths:
  /buildings:
    get:
      summary: List buildings
      operationId: listBuildings
      tags: [Buildings]
      parameters:
        - name: limit
          in: query
          schema:
            type: integer
            default: 100
      responses:
        '200':
          description: List of buildings
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/BuildingList'

components:
  securitySchemes:
    oauth2:
      type: oauth2
      flows:
        authorizationCode:
          authorizationUrl: https://auth.example.com/oauth/authorize
          tokenUrl: https://auth.example.com/oauth/token
          scopes:
            buildings:read: Read building information
            energy:read: Read energy data
```

---

**Chapter Summary:**

This chapter covered the API Interface specification (Phase 2):

- RESTful API design with resource-oriented endpoints
- OAuth 2.0 authentication with multiple grant types
- Comprehensive RBAC with fine-grained permissions
- Energy, equipment, and environmental data endpoints
- Real-time streaming via WebSocket and SSE
- Error handling and rate limiting strategies
- TypeScript and Python SDK implementations

In the next chapter, we will explore the Control Protocols (Phase 3), covering automated control sequences, optimization algorithms, and fault detection.

---

© 2025 World Certification Industry Association (WIA)
弘益人間 (Hongik Ingan) - Benefit All Humanity
