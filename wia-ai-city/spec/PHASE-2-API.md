# WIA AI-City - Phase 2: API Interface

> **Version:** 1.0.0
> **Status:** Complete
> **Last Updated:** 2025-12-24

---

## 1. Overview

This document defines the API interfaces for WIA AI-City, including:
- City Management API
- Rust-Core Control API
- Sharing Engine API
- Data Sovereignty API
- AIDC-Link API

Base URL: `https://api.ai-city.wia.org/v1`

---

## 2. Authentication

### 2.1 API Key Authentication

```http
Authorization: Bearer <API_KEY>
```

### 2.2 Mutual TLS (mTLS)

For production environments, mTLS is required:
- Client certificate issued by WIA CA
- TLS 1.3 minimum
- Ed25519 or P-384 key pairs

---

## 3. City Management API

### 3.1 Create AI-City Instance

```http
POST /cities
Content-Type: application/json

{
  "city": {
    "name": "Seoul AI-City",
    "population": 10000000,
    "type": "smart",
    "location": {
      "lat": 37.5665,
      "lng": 126.9780,
      "country": "KR"
    }
  },
  "rustCore": {
    "hbm_gb": 32,
    "power_watts": 2000
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "data": {
    "id": "AIC-2024-SEOUL1",
    "city": { ... },
    "rustCore": { ... },
    "sharingEngine": { ... },
    "dataSovereignty": { ... },
    "aidcLink": { ... },
    "metadata": {
      "created": "2024-12-24T10:00:00Z",
      "status": "active"
    }
  }
}
```

### 3.2 Get AI-City Instance

```http
GET /cities/{city_id}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "id": "AIC-2024-SEOUL1",
    "city": { ... },
    "rustCore": { ... },
    "metadata": { ... }
  }
}
```

### 3.3 Update AI-City Instance

```http
PATCH /cities/{city_id}
Content-Type: application/json

{
  "rustCore": {
    "power_watts": 2500
  }
}
```

### 3.4 List AI-Cities

```http
GET /cities?page=1&limit=20&status=active
```

---

## 4. Rust-Core Control API

### 4.1 Get Rust-Core Status

```http
GET /cities/{city_id}/rust-core/status
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "city_id": "AIC-2024-SEOUL1",
    "timestamp": "2024-12-24T10:30:00Z",
    "hbm": {
      "capacity_gb": 32,
      "used_gb": 24.5,
      "bandwidth_tbps": 38.4,
      "active_bandwidth_tbps": 28.8
    },
    "power": {
      "budget_watts": 2000,
      "current_watts": 1650,
      "ai_watts": 1400,
      "infra_watts": 400,
      "reserve_watts": 200
    },
    "thermal": {
      "core_temp_celsius": 62.5,
      "ambient_temp_celsius": 22.0,
      "target_temp_celsius": 25.0,
      "throttle_level": "NONE",
      "cooling_status": "optimal"
    }
  }
}
```

### 4.2 Adjust Power Allocation

```http
POST /cities/{city_id}/rust-core/power
Content-Type: application/json

{
  "ai_allocation": 0.75,
  "infra_allocation": 0.18,
  "reserve_allocation": 0.07
}
```

### 4.3 Set Thermal Target

```http
POST /cities/{city_id}/rust-core/thermal
Content-Type: application/json

{
  "target_temp_celsius": 23.0,
  "cooling_mode": "aggressive"
}
```

### 4.4 HBM Memory Management

```http
GET /cities/{city_id}/rust-core/hbm/usage
```

```http
POST /cities/{city_id}/rust-core/hbm/allocate
Content-Type: application/json

{
  "workload_id": "traffic-ml-model",
  "required_gb": 8,
  "priority": "high"
}
```

---

## 5. Sharing Engine API

### 5.1 Record Transaction

```http
POST /cities/{city_id}/sharing/transactions
Content-Type: application/json

{
  "amount": 10000.00,
  "currency": "USD",
  "merchant_id": "M-12345",
  "description": "City service payment"
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "transaction_id": "TXN-2024-ABC123DEF456",
    "amount": 10000.00,
    "contribution": {
      "total": 100.00,
      "education": 30.00,
      "healthcare": 30.00,
      "environment": 20.00,
      "emergency": 20.00
    },
    "blockchain_ref": "0x7f8e9d..."
  }
}
```

### 5.2 Get Transaction

```http
GET /cities/{city_id}/sharing/transactions/{transaction_id}
```

### 5.3 Get Transparency Report

```http
GET /cities/{city_id}/sharing/reports?period=2024-12
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "city_id": "AIC-2024-SEOUL1",
    "period": {
      "start": "2024-12-01",
      "end": "2024-12-31"
    },
    "total_transactions": 1250000,
    "total_volume": 5000000000.00,
    "total_contribution": 50000000.00,
    "distribution": {
      "education": {
        "amount": 15000000.00,
        "beneficiaries": 50000
      },
      "healthcare": {
        "amount": 15000000.00,
        "beneficiaries": 75000
      },
      "environment": {
        "amount": 10000000.00,
        "projects": 150
      },
      "emergency": {
        "amount": 10000000.00,
        "reserve_total": 250000000.00
      }
    },
    "verification_hash": "sha256:abc123..."
  }
}
```

### 5.4 Get Fund Balance

```http
GET /cities/{city_id}/sharing/funds/{fund_type}
```

---

## 6. Data Sovereignty API

### 6.1 Request Consent

```http
POST /cities/{city_id}/sovereignty/consent/request
Content-Type: application/json

{
  "citizen_id": "CIT-2024-KR-12345",
  "data_type": "L2",
  "purpose": "Traffic optimization",
  "scope": ["location", "travel_patterns"],
  "duration_days": 365
}
```

**Response:**
```json
{
  "status": "pending",
  "data": {
    "request_id": "REQ-ABC123",
    "citizen_portal_url": "https://consent.ai-city.wia.org/r/ABC123",
    "expires_at": "2024-12-25T10:00:00Z"
  }
}
```

### 6.2 Grant Consent (Citizen Action)

```http
POST /sovereignty/consent/grant
Content-Type: application/json

{
  "request_id": "REQ-ABC123",
  "citizen_signature": "Ed25519:...",
  "granted_scope": ["location"]
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "token_id": "CST-A1B2C3D4E5F6G7H8",
    "data_type": "L2",
    "scope": ["location"],
    "granted_at": "2024-12-24T11:00:00Z",
    "expires_at": "2025-12-24T11:00:00Z"
  }
}
```

### 6.3 Revoke Consent

```http
DELETE /sovereignty/consent/{token_id}
```

### 6.4 List Active Consents

```http
GET /sovereignty/citizens/{citizen_id}/consents
```

### 6.5 Request Data Export

```http
POST /sovereignty/citizens/{citizen_id}/export
Content-Type: application/json

{
  "data_types": ["L1", "L2", "L3"],
  "format": "JSON",
  "encryption": {
    "enabled": true,
    "public_key": "-----BEGIN PUBLIC KEY-----..."
  }
}
```

**Response:**
```json
{
  "status": "processing",
  "data": {
    "request_id": "EXP-12345",
    "estimated_completion": "2024-12-24T12:00:00Z",
    "status_url": "/sovereignty/export/EXP-12345"
  }
}
```

### 6.6 Request Data Erasure

```http
POST /sovereignty/citizens/{citizen_id}/erasure
Content-Type: application/json

{
  "data_types": ["L2", "L3"],
  "reason": "citizen_request",
  "verification": {
    "method": "biometric",
    "token": "..."
  }
}
```

---

## 7. AIDC-Link API

### 7.1 Send Message

```http
POST /cities/{city_id}/aidc-link/messages
Content-Type: application/json

{
  "layer": 4,
  "message_type": "CONTROL_COMMAND",
  "destination": {
    "node_id": "AIDC-NODE-042",
    "type": "controller"
  },
  "payload": {
    "service": "traffic-management",
    "action": "adjust_signal",
    "parameters": {
      "signal_id": "SIG-A7-001",
      "green_duration": 45
    }
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "message_id": "MSG-2024-XYZ789",
    "delivered": true,
    "latency_ms": 12
  }
}
```

### 7.2 Subscribe to Node Events

```http
POST /cities/{city_id}/aidc-link/subscribe
Content-Type: application/json

{
  "node_id": "AIDC-NODE-001",
  "message_types": ["SENSOR_DATA", "ALERT"],
  "webhook_url": "https://my-service.example.com/webhook"
}
```

### 7.3 Get Node Status

```http
GET /cities/{city_id}/aidc-link/nodes/{node_id}
```

### 7.4 List All Nodes

```http
GET /cities/{city_id}/aidc-link/nodes?type=sensor&zone=A7
```

### 7.5 WebSocket Connection

```javascript
const ws = new WebSocket('wss://api.ai-city.wia.org/v1/cities/AIC-2024-SEOUL1/aidc-link/stream');

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  console.log('Received:', message);
};

ws.send(JSON.stringify({
  action: 'subscribe',
  topics: ['traffic', 'alerts']
}));
```

---

## 8. Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `AIC_001` | 400 | Invalid request format |
| `AIC_002` | 401 | Authentication required |
| `AIC_003` | 403 | Insufficient permissions |
| `AIC_004` | 404 | Resource not found |
| `AIC_005` | 409 | Resource conflict |
| `AIC_006` | 422 | Validation error |
| `AIC_007` | 429 | Rate limit exceeded |
| `AIC_008` | 500 | Internal server error |
| `AIC_009` | 503 | Service unavailable |

**Error Response Format:**
```json
{
  "status": "error",
  "error": {
    "code": "AIC_004",
    "message": "AI-City instance not found",
    "details": {
      "city_id": "AIC-2024-INVALID"
    }
  }
}
```

---

## 9. Rate Limiting

| Tier | Requests/min | Burst |
|------|-------------|-------|
| Free | 60 | 100 |
| Standard | 600 | 1000 |
| Enterprise | 6000 | 10000 |
| Government | Unlimited | Unlimited |

**Rate Limit Headers:**
```http
X-RateLimit-Limit: 600
X-RateLimit-Remaining: 599
X-RateLimit-Reset: 1703419200
```

---

## 10. SDK Examples

### TypeScript

```typescript
import { AICityClient } from '@wia/ai-city-sdk';

const client = new AICityClient({
  apiKey: 'your-api-key',
  baseUrl: 'https://api.ai-city.wia.org/v1'
});

// Create AI-City
const city = await client.cities.create({
  name: 'New Smart City',
  population: 500000,
  type: 'eco',
  rustCore: { hbm_gb: 16, power_watts: 1000 }
});

// Get Rust-Core status
const status = await client.rustCore.getStatus(city.id);
console.log(`Temperature: ${status.thermal.core_temp_celsius}°C`);

// Record transaction with sharing
const tx = await client.sharing.recordTransaction(city.id, {
  amount: 1000,
  currency: 'USD'
});
console.log(`Contribution: $${tx.contribution.total}`);
```

### Python

```python
from wia_ai_city import AICityClient

client = AICityClient(api_key="your-api-key")

# Create AI-City
city = client.cities.create(
    name="New Smart City",
    population=500000,
    city_type="eco",
    rust_core={"hbm_gb": 16, "power_watts": 1000}
)

# Get status
status = client.rust_core.get_status(city.id)
print(f"Temperature: {status.thermal.core_temp_celsius}°C")

# Subscribe to events
@client.aidc_link.on("SENSOR_DATA")
def handle_sensor(message):
    print(f"Sensor: {message.payload}")
```

### Rust

```rust
use wia_ai_city::{Client, CityConfig, RustCoreConfig};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = Client::new("your-api-key");

    // Create AI-City
    let city = client.cities().create(CityConfig {
        name: "New Smart City".into(),
        population: 500000,
        city_type: CityType::Eco,
        rust_core: RustCoreConfig {
            hbm_gb: 16,
            power_watts: 1000,
            ..Default::default()
        },
    }).await?;

    // Get status
    let status = client.rust_core().status(&city.id).await?;
    println!("Temperature: {}°C", status.thermal.core_temp_celsius);

    Ok(())
}
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
