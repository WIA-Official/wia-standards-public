# WIA-SEMI-001: Phase 2 - API Interface Standards

Version: 1.0
Status: Final
Date: 2025-01-15

## Overview

Phase 2 defines standardized software interfaces for semiconductor control and monitoring. Building on Phase 1's data formats, Phase 2 enables unified programmatic access to chip functionality through RESTful APIs, driver interfaces, and SDKs.

## Design Principles

1. **RESTful Architecture**: HTTP-based APIs using standard methods
2. **Stateless**: Each request contains all necessary information
3. **Resource-Oriented**: Expose chip capabilities as resources
4. **Versioned**: API versions in URL path
5. **Self-Documenting**: OpenAPI/Swagger specifications
6. **Security-First**: Built-in authentication and authorization

## Base API Structure

### URL Format

```
https://{chip-hostname}/api/v{version}/{resource}/{action}
```

Examples:
- `https://soc.local/api/v1/chip/info`
- `https://192.168.1.100/api/v1/monitor/temperature`

### API Versioning

- Version in URL path: `/api/v1/`
- Current version: v1
- Future versions: v2, v3, etc.
- Backward compatibility maintained for 2 major versions

## Core API Endpoints

### 1. Information APIs (Read-Only)

#### GET /api/v1/chip/info

Returns complete chip specification in WIA-SEMI-001 format.

**Request:**
```http
GET /api/v1/chip/info HTTP/1.1
Host: soc.local
Authorization: Bearer {token}
```

**Response:**
```json
{
  "wiaVersion": "1.0",
  "metadata": {
    "id": "WIA-SOC-2025-001",
    "productName": "ExampleSoC Gen 5"
  },
  "architecture": { ... },
  "power": { ... }
}
```

#### GET /api/v1/chip/capabilities

Lists available features and optional capabilities.

**Response:**
```json
{
  "cpu": {
    "cores": 8,
    "features": ["DVFS", "power-gating", "SMT"]
  },
  "gpu": {
    "present": true,
    "features": ["ray-tracing", "VRS"]
  },
  "npu": {
    "present": true,
    "performance": {"value": 50, "unit": "TOPS"}
  }
}
```

#### GET /api/v1/chip/architecture

Detailed architecture information.

#### GET /api/v1/chip/interfaces

Available I/O interfaces and their configurations.

### 2. Configuration APIs

#### PUT /api/v1/config/power-mode

Set power/performance mode.

**Request:**
```http
PUT /api/v1/config/power-mode HTTP/1.1
Content-Type: application/json

{
  "mode": "performance",
  "sustainedDuration": 60,
  "autoRevert": true
}
```

**Modes:**
- `efficiency`: Minimum power consumption
- `balanced`: Balance between power and performance
- `performance`: Maximum performance

**Response:**
```json
{
  "status": "success",
  "currentMode": "performance",
  "effectiveAt": "2025-01-15T10:30:00Z",
  "estimatedPower": {"value": 10.5, "unit": "W"}
}
```

#### PUT /api/v1/config/frequency

Set frequency limits for CPU/GPU.

**Request:**
```json
{
  "cpu": {
    "min": 1000,
    "max": 3500,
    "unit": "MHz"
  },
  "gpu": {
    "min": 400,
    "max": 1000,
    "unit": "MHz"
  }
}
```

#### PUT /api/v1/config/thermal-limit

Set thermal throttling threshold.

**Request:**
```json
{
  "threshold": 90,
  "unit": "°C",
  "action": "gradual-throttle"
}
```

#### PUT /api/v1/config/power-limit

Set maximum power consumption.

**Request:**
```json
{
  "limit": 10,
  "unit": "W",
  "enforcement": "hard"
}
```

### 3. Monitoring APIs

#### GET /api/v1/monitor/power

Current power consumption.

**Response:**
```json
{
  "timestamp": "2025-01-15T10:30:15.234Z",
  "total": {"value": 8.5, "unit": "W"},
  "breakdown": {
    "cpu": {"value": 4.2, "unit": "W"},
    "gpu": {"value": 3.1, "unit": "W"},
    "other": {"value": 1.2, "unit": "W"}
  }
}
```

#### GET /api/v1/monitor/temperature

Temperature from all sensors.

**Response:**
```json
{
  "timestamp": "2025-01-15T10:30:15.234Z",
  "sensors": [
    {
      "location": "CPU-Core0",
      "temperature": {"value": 72.5, "unit": "°C"},
      "status": "normal"
    },
    {
      "location": "GPU",
      "temperature": {"value": 68.2, "unit": "°C"},
      "status": "normal"
    }
  ]
}
```

#### GET /api/v1/monitor/frequency

Current operating frequencies.

**Response:**
```json
{
  "timestamp": "2025-01-15T10:30:15.234Z",
  "cpu": {
    "cluster0": [3200, 3150],
    "cluster1": [1800, 1750, 1800, 1750],
    "unit": "MHz"
  },
  "gpu": {"value": 950, "unit": "MHz"}
}
```

#### GET /api/v1/monitor/utilization

Component utilization percentages.

**Response:**
```json
{
  "timestamp": "2025-01-15T10:30:15.234Z",
  "cpu": {
    "overall": 45,
    "perCore": [85, 82, 15, 12, 20, 18, 25, 22]
  },
  "gpu": 78,
  "npu": 0,
  "memory": 62,
  "unit": "%"
}
```

#### GET /api/v1/monitor/counters

Performance counters and statistics.

## WebSocket Streaming API

For high-frequency monitoring (up to 1000 Hz):

### Connection

```javascript
const ws = new WebSocket('wss://soc.local/api/v1/stream/telemetry');
```

### Subscribe

```javascript
ws.send(JSON.stringify({
  action: 'subscribe',
  metrics: ['power', 'temperature', 'frequency'],
  sampleRate: 100 // Hz
}));
```

### Receive Data

```javascript
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  // {timestamp, power, temperature, frequency}
};
```

## Authentication and Security

### Supported Methods

1. **API Key** (Development)
   ```http
   Authorization: ApiKey your-api-key
   ```

2. **OAuth 2.0** (Production)
   ```http
   Authorization: Bearer eyJhbGciOiJSUzI1NiIs...
   ```

3. **JWT Token** (Recommended)
   ```http
   Authorization: Bearer eyJhbGciOiJIUzI1NiIs...
   ```

4. **Mutual TLS** (High Security)
   - Client certificate required
   - Certificate-based authentication

### Authorization Scopes

- `chip:read` - Read chip information
- `chip:monitor` - Access monitoring data
- `chip:config` - Modify configuration
- `chip:debug` - Debug features
- `chip:admin` - Full access

### OAuth 2.0 Token Request

```http
POST /oauth/token HTTP/1.1
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=your_client_id
&client_secret=your_client_secret
&scope=chip:read chip:monitor
```

## Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Frequency exceeds maximum",
    "details": {
      "parameter": "maxFrequency",
      "requestedValue": 4500,
      "maximumValue": 3800
    },
    "timestamp": "2025-01-15T10:30:15Z",
    "requestId": "req_1234567890"
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_PARAMETER | 400 | Invalid request parameter |
| UNAUTHORIZED | 401 | Missing/invalid authentication |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource doesn't exist |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| THERMAL_THROTTLED | 503 | Operation unavailable due to thermal state |

## Driver Interfaces

### Linux Driver

```c
#include <linux/wia-semi.h>

struct wia_semi_chip_info {
    const char *manufacturer;
    const char *product_name;
    const char *chip_id;
};

struct wia_semi_pm_ops {
    int (*set_power_mode)(struct device *dev, enum wia_power_mode mode);
    int (*get_power_consumption)(struct device *dev, u32 *milliwatts);
};

int wia_semi_register_driver(struct wia_semi_driver *driver);
```

### Windows Driver

```c
#include <wiasemi.h>

typedef struct _WIA_SEMI_CHIP_INFO {
    UNICODE_STRING Manufacturer;
    UNICODE_STRING ProductName;
    UNICODE_STRING ChipId;
} WIA_SEMI_CHIP_INFO;

#define IOCTL_WIA_SEMI_GET_INFO \
    CTL_CODE(FILE_DEVICE_UNKNOWN, 0x800, METHOD_BUFFERED, FILE_ANY_ACCESS)
```

## SDK Reference

### TypeScript SDK

```typescript
import { WIASemiChip, PowerMode } from '@wia/semi-sdk';

const chip = new WIASemiChip({
  host: 'soc.local',
  apiKey: process.env.WIA_API_KEY
});

// Get info
const info = await chip.getInfo();

// Set power mode
await chip.config.setPowerMode(PowerMode.Performance);

// Monitor telemetry
chip.monitor.on('telemetry', (data) => {
  console.log(`Power: ${data.power.value}W`);
});
```

### Python SDK

```python
from wia_semi import WIASemiChip, PowerMode

chip = WIASemiChip(host='soc.local', api_key=os.environ['WIA_API_KEY'])
info = chip.get_info()
chip.config.set_power_mode(PowerMode.PERFORMANCE)

for telemetry in chip.monitor.stream_telemetry(sample_rate=100):
    print(f"Power: {telemetry['power']['value']}W")
```

## Rate Limiting

- Information APIs: 100 requests/minute
- Configuration APIs: 10 requests/minute
- Monitoring APIs: 1000 requests/minute
- WebSocket streaming: 1 connection per client

## Compliance Requirements

To be Phase 2 compliant:

1. ✓ Implement all Information APIs
2. ✓ Implement Configuration APIs (power-mode required, others optional)
3. ✓ Implement Monitoring APIs (power, temperature required)
4. ✓ Support at least one authentication method
5. ✓ Provide OpenAPI specification
6. ✓ Implement standard error responses
7. ✓ Support API versioning

## Testing

Use the official test suite:

```bash
npm install -g wia-semi-api-test
wia-semi-api-test --host soc.local --api-key your-key
```

---

**Previous**: [Phase 1 - Data Format](PHASE-1-DATA-FORMAT.md)
**Next**: [Phase 3 - Protocol Implementation](PHASE-3-PROTOCOL.md)

© 2025 SmileStory Inc. / WIA
弘益人間 - Benefit All Humanity
