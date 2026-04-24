# WIA-BATTERY-PASSPORT: Phase 3 - Communication Protocol

**EU 배터리 여권 통신 프로토콜**
*API, BMS integration, and data exchange*

홍익인간 (弘益人間) - Benefit All Humanity

---

## 1. Overview

This document specifies:
- REST API for battery passport management
- BMS (Battery Management System) integration protocol
- Real-time data streaming via WebSocket
- QR code scanning and data retrieval
- Interoperability with EU Battery Passport ecosystem

---

## 2. REST API

### 2.1 Base URL

```
Production: https://api.battery.wia.org/v1
Staging:    https://api-staging.battery.wia.org/v1
```

### 2.2 Authentication

```http
Authorization: Bearer <access_token>
X-WIA-API-Key: <api_key>
```

OAuth 2.0 scopes:
- `passport:read` - Read passport data
- `passport:write` - Create/update passports
- `health:read` - Read health data
- `health:write` - Update health data (BMS only)
- `lifecycle:write` - Add lifecycle events
- `admin` - Administrative operations

### 2.3 Endpoints

#### Battery Passport CRUD

```http
# Create new passport
POST /passports
Content-Type: application/json

{
  "identity": { ... },
  "manufacturer": { ... },
  "specifications": { ... },
  "materials": { ... },
  "carbon_footprint": { ... }
}

Response: 201 Created
{
  "passport_id": "01935a2b-3c4d-7e8f-9a0b-1c2d3e4f5a6b",
  "qr_code": "WIABAT:ABC123DEF456GHI7",
  "created_at": "2025-12-16T10:30:00Z"
}
```

```http
# Get passport by ID
GET /passports/{passport_id}

Response: 200 OK
{
  "id": "01935a2b-3c4d-7e8f-9a0b-1c2d3e4f5a6b",
  "version": "1.0.0",
  "identity": { ... },
  ...
}
```

```http
# Update passport
PUT /passports/{passport_id}
Content-Type: application/json

{
  "health": { ... }
}

Response: 200 OK
```

```http
# List passports (with filters)
GET /passports?manufacturer=CATL&chemistry=nmc&page=1&limit=50

Response: 200 OK
{
  "passports": [...],
  "total": 1250,
  "page": 1,
  "limit": 50
}
```

#### Health Data

```http
# Get current health status
GET /passports/{passport_id}/health

Response: 200 OK
{
  "state_of_health_percent": 94.5,
  "state_of_charge_percent": 78.2,
  "current_capacity_ah": 189.0,
  "internal_resistance_mohm": 12.5,
  "total_energy_throughput_kwh": 45230,
  "full_cycle_equivalents": 452,
  "last_bms_sync": "2025-12-16T09:45:00Z"
}
```

```http
# Update health data (BMS integration)
POST /passports/{passport_id}/health
Authorization: Bearer <bms_token>
Content-Type: application/json

{
  "state_of_health_percent": 94.3,
  "state_of_charge_percent": 82.1,
  "current_capacity_ah": 188.6,
  "internal_resistance_mohm": 12.7,
  "cell_voltages": [3.65, 3.64, 3.65, ...],
  "cell_temperatures": [28.5, 29.1, 28.8, ...],
  "total_energy_throughput_kwh": 45350,
  "recorded_at": "2025-12-16T10:00:00Z"
}

Response: 201 Created
```

```http
# Get health history
GET /passports/{passport_id}/health/history?from=2025-01-01&to=2025-12-16

Response: 200 OK
{
  "history": [
    {
      "timestamp": "2025-12-16T00:00:00Z",
      "soh": 94.5,
      "soc": 45.2
    },
    ...
  ]
}
```

#### Lifecycle Events

```http
# Add lifecycle event
POST /passports/{passport_id}/lifecycle
Content-Type: application/json

{
  "event_type": "maintained",
  "timestamp": "2025-12-15T14:30:00Z",
  "country": "DE",
  "actor": {
    "type": "service_center",
    "name": "BMW Service Berlin",
    "id": "DE-BMW-001"
  },
  "data": {
    "maintenance_type": "inspection",
    "description": "Annual battery health check",
    "soh_before": 94.5,
    "soh_after": 94.5
  }
}

Response: 201 Created
{
  "event_id": "evt_abc123",
  "verified": false
}
```

```http
# Get lifecycle history
GET /passports/{passport_id}/lifecycle

Response: 200 OK
{
  "events": [
    {
      "id": "evt_abc123",
      "event_type": "manufactured",
      "timestamp": "2024-06-15T08:00:00Z",
      ...
    },
    ...
  ]
}
```

#### Carbon Footprint

```http
# Get carbon footprint
GET /passports/{passport_id}/carbon

Response: 200 OK
{
  "total_kg_co2e": 5250.50,
  "per_kwh_kg_co2e": 70.0,
  "performance_class": "C",
  "breakdown": {
    "raw_material_acquisition": 3150.30,
    "manufacturing": 1575.15,
    "transport": 525.05
  },
  "third_party_verified": true,
  "verifier": "TÜV Rheinland"
}
```

```http
# Calculate/update carbon footprint
POST /passports/{passport_id}/carbon/calculate

Response: 200 OK
{
  "total_kg_co2e": 5250.50,
  "calculation_date": "2025-12-16T10:30:00Z"
}
```

#### SOH Calculation

```http
# Calculate current SOH
GET /passports/{passport_id}/soh/calculate

Response: 200 OK
{
  "soh_percent": 94.5,
  "capacity_soh": 94.5,
  "resistance_soh": 95.0,
  "cycle_soh": 92.0,
  "confidence": "high",
  "measurement_date": "2025-12-16T10:30:00Z"
}
```

#### Remaining Useful Life

```http
# Predict RUL
GET /passports/{passport_id}/rul/predict?usage_profile=ev_moderate

Response: 200 OK
{
  "remaining_months": 72,
  "remaining_cycles": 1500,
  "expected_eol_date": "2031-12-16",
  "confidence": "medium",
  "methodology": "arrhenius_adjusted"
}
```

#### Second-Life Assessment

```http
# Assess second-life eligibility
GET /passports/{passport_id}/second-life/assess

Response: 200 OK
{
  "eligible": true,
  "score": 85,
  "suggested_applications": [
    "Grid-scale energy storage",
    "Residential energy storage"
  ],
  "estimated_remaining_value": {
    "usd": 4500.00,
    "per_kwh_usd": 60.00
  },
  "checks": [
    {"name": "soh_range", "passed": true, "message": "SOH 78% in acceptable range"},
    ...
  ]
}
```

#### Recycling

```http
# Get recycling information
GET /passports/{passport_id}/recycling

Response: 200 OK
{
  "recyclability_score": 85,
  "design_for_recycling": true,
  "disassembly_manual_url": "https://...",
  "recovery_targets": {
    "cobalt": 95,
    "lithium": 80,
    "nickel": 95
  }
}
```

```http
# Record recycling event
POST /passports/{passport_id}/recycling/record
Content-Type: application/json

{
  "recycler_license": "DE-REC-2024-0001",
  "process_type": "hydrometallurgical",
  "materials_recovered": [
    {"material": "cobalt", "weight_kg": 12.5, "purity_percent": 99.5},
    {"material": "lithium", "weight_kg": 8.2, "purity_percent": 98.0},
    {"material": "nickel", "weight_kg": 45.0, "purity_percent": 99.0}
  ],
  "recycling_date": "2025-12-15"
}

Response: 201 Created
```

#### QR Code Lookup

```http
# Lookup by QR code
GET /passports/qr/{qr_code}

Example: GET /passports/qr/WIABAT:ABC123DEF456GHI7

Response: 200 OK
{
  "passport_id": "01935a2b-3c4d-7e8f-9a0b-1c2d3e4f5a6b",
  "identity": { ... },
  ...
}
```

#### Supply Chain Verification

```http
# Verify supply chain
POST /passports/{passport_id}/supply-chain/verify

Response: 200 OK
{
  "overall_risk_score": 25.0,
  "eu_due_diligence_compliant": true,
  "material_verifications": [
    {
      "material": "cobalt",
      "source_countries": ["AU"],
      "country_risk_level": "LOW",
      "certification_valid": true
    },
    ...
  ]
}
```

---

## 3. WebSocket API

### 3.1 Connection

```javascript
const ws = new WebSocket('wss://api.battery.wia.org/v1/ws');

ws.onopen = () => {
  // Authenticate
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'Bearer <access_token>'
  }));

  // Subscribe to passport updates
  ws.send(JSON.stringify({
    type: 'subscribe',
    passport_id: '01935a2b-3c4d-7e8f-9a0b-1c2d3e4f5a6b',
    events: ['health_update', 'lifecycle_event', 'alert']
  }));
};
```

### 3.2 Event Types

```typescript
// Health update (from BMS)
{
  "type": "health_update",
  "passport_id": "01935a2b-...",
  "timestamp": "2025-12-16T10:30:00Z",
  "data": {
    "soh_percent": 94.5,
    "soc_percent": 78.2,
    "temperature_c": 28.5,
    "power_w": 15000
  }
}

// Lifecycle event
{
  "type": "lifecycle_event",
  "passport_id": "01935a2b-...",
  "timestamp": "2025-12-16T10:30:00Z",
  "event": {
    "event_type": "charged",
    "data": { ... }
  }
}

// Alert
{
  "type": "alert",
  "passport_id": "01935a2b-...",
  "timestamp": "2025-12-16T10:30:00Z",
  "severity": "warning",
  "message": "High temperature detected (45°C)",
  "code": "TEMP_HIGH"
}

// SOH threshold crossed
{
  "type": "soh_threshold",
  "passport_id": "01935a2b-...",
  "timestamp": "2025-12-16T10:30:00Z",
  "previous_soh": 80.5,
  "current_soh": 79.8,
  "threshold": 80.0,
  "recommendation": "Consider second-life assessment"
}
```

---

## 4. BMS Integration Protocol

### 4.1 Overview

Direct integration with Battery Management Systems for real-time health data.

### 4.2 BMS Registration

```http
POST /bms/register
Content-Type: application/json

{
  "passport_id": "01935a2b-...",
  "bms_manufacturer": "Bosch",
  "bms_model": "BMS-500",
  "firmware_version": "2.3.1",
  "communication_protocol": "CAN",
  "vehicle_vin": "WBA12345678901234"
}

Response: 201 Created
{
  "bms_id": "bms_xyz789",
  "api_key": "bms_key_...",
  "upload_endpoint": "https://api.battery.wia.org/v1/bms/upload"
}
```

### 4.3 Data Upload (from BMS)

```http
POST /bms/upload
Authorization: X-BMS-Key <bms_api_key>
Content-Type: application/json

{
  "bms_id": "bms_xyz789",
  "passport_id": "01935a2b-...",
  "recorded_at": "2025-12-16T10:30:00Z",

  // Pack level
  "pack_voltage_v": 400.5,
  "pack_current_a": 125.0,
  "pack_power_w": 50062.5,
  "pack_temperature_c": 32.5,

  // Cell level (array per cell)
  "cell_count": 96,
  "cell_voltages_v": [3.65, 3.64, 3.65, ...],
  "cell_temperatures_c": [32.1, 32.3, 32.0, ...],
  "cell_balancing_active": [false, false, true, ...],

  // State
  "soc_percent": 78.2,
  "soh_percent": 94.5,

  // Counters
  "total_energy_charged_kwh": 24500.5,
  "total_energy_discharged_kwh": 23800.2,
  "charge_cycles": 452,

  // Faults
  "active_faults": [],
  "fault_history": []
}

Response: 201 Created
```

### 4.4 CAN Bus Protocol

For direct CAN integration:

```
CAN ID: 0x6B0 - Battery Status
  Byte 0-1: SOC (0.1% resolution)
  Byte 2-3: SOH (0.1% resolution)
  Byte 4-5: Pack voltage (0.1V resolution)
  Byte 6-7: Pack current (0.1A resolution, signed)

CAN ID: 0x6B1 - Cell Voltages (broadcast sequentially)
  Byte 0: Cell group index (0-11)
  Byte 1-2: Cell 1 voltage (0.001V resolution)
  Byte 3-4: Cell 2 voltage
  Byte 5-6: Cell 3 voltage
  Byte 7: Reserved

CAN ID: 0x6B2 - Temperatures
  Byte 0: Sensor index
  Byte 1: Temperature (°C, offset -40)
  ...
```

---

## 5. QR Code Protocol

### 5.1 QR Code Generation

```http
# Generate QR code image
GET /passports/{passport_id}/qr?format=png&size=300

Response: 200 OK
Content-Type: image/png
[PNG image data]
```

### 5.2 QR Code Content

**URL Format (online lookup):**
```
https://battery.wia.org/p/01935a2b-3c4d-7e8f-9a0b-1c2d3e4f5a6b
```

**Compact Format (offline capable):**
```
WIABAT:1.0:<base64url_data>
```

Decoded data structure:
```json
{
  "id": "01935a2b-3c4d",
  "mfr": "CATL",
  "mod": "NMC811-75",
  "chem": "nmc",
  "cap": 75000,
  "soh": 94,
  "carb": "C",
  "prod": "2024-06-15",
  "sig": "<signature>"
}
```

### 5.3 Mobile Scanning Flow

```
1. User scans QR code
2. App detects WIABAT: prefix
3. If online:
   - Fetch full passport from API
   - Display comprehensive data
4. If offline:
   - Decode compact data
   - Display essential information
   - Cache for later sync
```

---

## 6. EU Interoperability

### 6.1 Catena-X Integration

```http
# Export to Catena-X format
GET /passports/{passport_id}/export?format=catena-x

Response: 200 OK
{
  "@context": "https://w3id.org/2023/catenax/battery-pass/v1",
  "type": "BatteryPass",
  ...
}
```

### 6.2 Global Battery Alliance (GBA) Format

```http
# Export to GBA format
GET /passports/{passport_id}/export?format=gba

Response: 200 OK
{
  "gba_version": "1.0",
  "battery_id": {...},
  ...
}
```

### 6.3 EU Battery Passport Exchange

```http
# Register with EU central registry
POST /eu/register
Content-Type: application/json

{
  "passport_id": "01935a2b-...",
  "eu_unique_identifier": "EU-BAT-2024-0001234567"
}
```

---

## 7. Error Handling

### 7.1 Error Response Format

```json
{
  "error": {
    "code": "PASSPORT_NOT_FOUND",
    "message": "Battery passport not found",
    "details": {
      "passport_id": "01935a2b-..."
    },
    "request_id": "req_abc123"
  }
}
```

### 7.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `PASSPORT_NOT_FOUND` | 404 | Passport ID not found |
| `INVALID_QR_CODE` | 400 | QR code format invalid |
| `BMS_NOT_AUTHORIZED` | 403 | BMS not registered for passport |
| `HEALTH_DATA_STALE` | 409 | Health data too old |
| `CARBON_UNVERIFIED` | 422 | Carbon footprint not verified |
| `SUPPLY_CHAIN_INCOMPLETE` | 422 | Supply chain data missing |
| `RECYCLING_INVALID` | 422 | Recycling data validation failed |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `EU_COMPLIANCE_FAILED` | 422 | Does not meet EU requirements |

---

## 8. Rate Limiting

```http
# Response headers
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1702720800
```

| Endpoint Category | Rate Limit |
|-------------------|------------|
| Read operations | 1000/hour |
| Write operations | 100/hour |
| BMS uploads | 60/minute |
| QR lookups | 500/hour |
| Export operations | 50/hour |

---

## 9. Webhooks

### 9.1 Webhook Registration

```http
POST /webhooks
Content-Type: application/json

{
  "url": "https://your-server.com/webhook",
  "events": ["health_update", "soh_threshold", "lifecycle_event"],
  "passport_ids": ["01935a2b-..."],
  "secret": "your_webhook_secret"
}
```

### 9.2 Webhook Payload

```http
POST https://your-server.com/webhook
Content-Type: application/json
X-WIA-Signature: sha256=...

{
  "event": "soh_threshold",
  "timestamp": "2025-12-16T10:30:00Z",
  "data": {
    "passport_id": "01935a2b-...",
    "previous_soh": 80.5,
    "current_soh": 79.8,
    "threshold": 80.0
  }
}
```

---

## 10. Security

### 10.1 Transport Security

- TLS 1.3 required
- Certificate pinning recommended for BMS integrations
- HSTS enabled

### 10.2 Data Encryption

- Health data encrypted at rest (AES-256-GCM)
- API keys hashed with Argon2
- BMS communication encrypted end-to-end

### 10.3 Audit Logging

All operations logged:
```json
{
  "timestamp": "2025-12-16T10:30:00Z",
  "actor": "bms_xyz789",
  "action": "health_update",
  "passport_id": "01935a2b-...",
  "ip_address": "203.0.113.42",
  "user_agent": "BMS-500/2.3.1"
}
```

---

**Document ID**: WIA-BATTERY-PASSPORT-PHASE-3
**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
