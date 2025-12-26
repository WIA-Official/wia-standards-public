# WIA AI-City - Phase 1: Data Format

> **Version:** 1.0.0
> **Status:** Complete
> **Last Updated:** 2025-12-24

---

## 1. Overview

This document defines the data formats for the WIA AI-City standard, covering:
- Rust-Core control system data structures
- 1% Sharing Engine transaction formats
- Citizen Data Sovereignty schemas
- AIDC-Link protocol messages

---

## 2. Core Data Schemas

### 2.1 AI-City Instance

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "AICityInstance",
  "type": "object",
  "required": ["id", "city", "rustCore", "sharingEngine", "dataSovereignty", "aidcLink"],
  "properties": {
    "id": {
      "type": "string",
      "pattern": "^AIC-\\d{4}-[A-Z0-9]{6}$",
      "description": "Unique AI-City identifier"
    },
    "city": {
      "$ref": "#/$defs/CityInfo"
    },
    "rustCore": {
      "$ref": "#/$defs/RustCoreConfig"
    },
    "sharingEngine": {
      "$ref": "#/$defs/SharingEngineConfig"
    },
    "dataSovereignty": {
      "$ref": "#/$defs/DataSovereigntyConfig"
    },
    "aidcLink": {
      "$ref": "#/$defs/AIDCLinkConfig"
    },
    "metadata": {
      "$ref": "#/$defs/Metadata"
    }
  },
  "$defs": {
    "CityInfo": {
      "type": "object",
      "required": ["name", "population", "type"],
      "properties": {
        "name": { "type": "string" },
        "population": { "type": "integer", "minimum": 10000 },
        "type": {
          "type": "string",
          "enum": ["smart", "industrial", "eco", "innovation"]
        },
        "location": {
          "type": "object",
          "properties": {
            "lat": { "type": "number" },
            "lng": { "type": "number" },
            "country": { "type": "string" }
          }
        }
      }
    },
    "RustCoreConfig": {
      "type": "object",
      "required": ["hbm_gb", "power_watts", "thermal_target"],
      "properties": {
        "hbm_gb": { "type": "integer", "minimum": 4, "maximum": 128 },
        "power_watts": { "type": "integer", "minimum": 100, "maximum": 10000 },
        "thermal_target": { "type": "number", "default": 25 },
        "ai_allocation": { "type": "number", "default": 0.7 },
        "infra_allocation": { "type": "number", "default": 0.2 },
        "reserve_allocation": { "type": "number", "default": 0.1 }
      }
    },
    "SharingEngineConfig": {
      "type": "object",
      "properties": {
        "contribution_rate": { "type": "number", "default": 0.01 },
        "education": { "type": "number", "default": 0.003 },
        "healthcare": { "type": "number", "default": 0.003 },
        "environment": { "type": "number", "default": 0.002 },
        "emergency": { "type": "number", "default": 0.002 }
      }
    },
    "DataSovereigntyConfig": {
      "type": "object",
      "properties": {
        "enabled": { "type": "boolean", "default": true },
        "consent_required": { "type": "boolean", "default": true },
        "portability": { "type": "boolean", "default": true },
        "right_to_erasure": { "type": "boolean", "default": true }
      }
    },
    "AIDCLinkConfig": {
      "type": "object",
      "properties": {
        "version": { "type": "string", "default": "1.0" },
        "layers": { "type": "integer", "default": 4 },
        "encryption": { "type": "string", "default": "AES-256-GCM" }
      }
    },
    "Metadata": {
      "type": "object",
      "properties": {
        "created": { "type": "string", "format": "date-time" },
        "updated": { "type": "string", "format": "date-time" },
        "status": {
          "type": "string",
          "enum": ["active", "maintenance", "suspended"]
        }
      }
    }
  }
}
```

### 2.2 Rust-Core Status

```json
{
  "title": "RustCoreStatus",
  "type": "object",
  "properties": {
    "city_id": { "type": "string" },
    "timestamp": { "type": "string", "format": "date-time" },
    "hbm": {
      "type": "object",
      "properties": {
        "capacity_gb": { "type": "number" },
        "used_gb": { "type": "number" },
        "bandwidth_tbps": { "type": "number" },
        "active_bandwidth_tbps": { "type": "number" }
      }
    },
    "power": {
      "type": "object",
      "properties": {
        "budget_watts": { "type": "number" },
        "current_watts": { "type": "number" },
        "ai_watts": { "type": "number" },
        "infra_watts": { "type": "number" },
        "reserve_watts": { "type": "number" }
      }
    },
    "thermal": {
      "type": "object",
      "properties": {
        "core_temp_celsius": { "type": "number" },
        "ambient_temp_celsius": { "type": "number" },
        "target_temp_celsius": { "type": "number" },
        "throttle_level": {
          "type": "string",
          "enum": ["NONE", "LOW", "MEDIUM", "HIGH"]
        },
        "cooling_status": { "type": "string" }
      }
    }
  }
}
```

---

## 3. Sharing Engine Data Formats

### 3.1 Transaction

```json
{
  "title": "SharingTransaction",
  "type": "object",
  "required": ["transaction_id", "amount", "contribution"],
  "properties": {
    "transaction_id": {
      "type": "string",
      "pattern": "^TXN-\\d{4}-[A-Z0-9]{12}$"
    },
    "city_id": { "type": "string" },
    "timestamp": { "type": "string", "format": "date-time" },
    "amount": {
      "type": "number",
      "minimum": 0
    },
    "currency": {
      "type": "string",
      "default": "USD"
    },
    "contribution": {
      "type": "object",
      "properties": {
        "total": { "type": "number" },
        "education": { "type": "number" },
        "healthcare": { "type": "number" },
        "environment": { "type": "number" },
        "emergency": { "type": "number" }
      }
    },
    "blockchain_ref": {
      "type": "string",
      "description": "Reference to blockchain ledger entry"
    }
  }
}
```

### 3.2 Transparency Report

```json
{
  "title": "TransparencyReport",
  "type": "object",
  "properties": {
    "city_id": { "type": "string" },
    "period": {
      "type": "object",
      "properties": {
        "start": { "type": "string", "format": "date" },
        "end": { "type": "string", "format": "date" }
      }
    },
    "total_transactions": { "type": "integer" },
    "total_volume": { "type": "number" },
    "total_contribution": { "type": "number" },
    "distribution": {
      "type": "object",
      "properties": {
        "education": {
          "type": "object",
          "properties": {
            "amount": { "type": "number" },
            "beneficiaries": { "type": "integer" }
          }
        },
        "healthcare": { "$ref": "#/properties/distribution/properties/education" },
        "environment": { "$ref": "#/properties/distribution/properties/education" },
        "emergency": { "$ref": "#/properties/distribution/properties/education" }
      }
    },
    "verification_hash": { "type": "string" }
  }
}
```

---

## 4. Citizen Data Sovereignty Formats

### 4.1 Data Classification

| Level | Code | Control | Description |
|-------|------|---------|-------------|
| 1 | L1 | Strict | Identity data (name, ID, biometrics) |
| 2 | L2 | Protected | Location data (GPS, address) |
| 3 | L3 | Optional | Behavioral data (shopping, travel) |
| 4 | L4 | Open | Public data (traffic, weather) |

### 4.2 Consent Token

```json
{
  "title": "ConsentToken",
  "type": "object",
  "required": ["token_id", "citizen_id", "data_type", "purpose"],
  "properties": {
    "token_id": {
      "type": "string",
      "pattern": "^CST-[A-Z0-9]{16}$"
    },
    "citizen_id": { "type": "string" },
    "data_type": {
      "type": "string",
      "enum": ["L1", "L2", "L3", "L4"]
    },
    "purpose": { "type": "string" },
    "requester": {
      "type": "object",
      "properties": {
        "entity_id": { "type": "string" },
        "name": { "type": "string" },
        "type": { "type": "string" }
      }
    },
    "scope": {
      "type": "array",
      "items": { "type": "string" }
    },
    "granted_at": { "type": "string", "format": "date-time" },
    "expires_at": { "type": "string", "format": "date-time" },
    "revocable": { "type": "boolean", "default": true },
    "status": {
      "type": "string",
      "enum": ["active", "revoked", "expired"]
    }
  }
}
```

### 4.3 Data Export Request

```json
{
  "title": "DataExportRequest",
  "type": "object",
  "properties": {
    "request_id": { "type": "string" },
    "citizen_id": { "type": "string" },
    "data_types": {
      "type": "array",
      "items": { "type": "string", "enum": ["L1", "L2", "L3", "L4"] }
    },
    "format": {
      "type": "string",
      "enum": ["JSON", "CSV", "XML"]
    },
    "encryption": {
      "type": "object",
      "properties": {
        "enabled": { "type": "boolean" },
        "public_key": { "type": "string" }
      }
    },
    "requested_at": { "type": "string", "format": "date-time" },
    "status": {
      "type": "string",
      "enum": ["pending", "processing", "completed", "failed"]
    }
  }
}
```

---

## 5. AIDC-Link Protocol Message Format

### 5.1 Base Message

```json
{
  "title": "AIDCLinkMessage",
  "type": "object",
  "required": ["aidc_version", "layer", "message_type", "source", "payload"],
  "properties": {
    "aidc_version": {
      "type": "string",
      "default": "1.0"
    },
    "layer": {
      "type": "integer",
      "minimum": 1,
      "maximum": 4
    },
    "message_type": {
      "type": "string",
      "enum": ["SENSOR_DATA", "CONTROL_COMMAND", "ALERT", "ANALYTICS", "HANDSHAKE", "HEARTBEAT"]
    },
    "source": {
      "$ref": "#/$defs/NodeInfo"
    },
    "destination": {
      "$ref": "#/$defs/NodeInfo"
    },
    "payload": {
      "type": "object"
    },
    "encryption": {
      "type": "string",
      "default": "AES-256-GCM"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "signature": {
      "type": "string",
      "description": "Ed25519 signature"
    }
  },
  "$defs": {
    "NodeInfo": {
      "type": "object",
      "properties": {
        "node_id": { "type": "string" },
        "type": {
          "type": "string",
          "enum": ["sensor", "controller", "gateway", "central", "edge"]
        },
        "location": {
          "type": "object",
          "properties": {
            "lat": { "type": "number" },
            "lng": { "type": "number" },
            "zone": { "type": "string" }
          }
        }
      }
    }
  }
}
```

### 5.2 Layer-Specific Payloads

#### Layer 1: Physical

```json
{
  "connection_type": "fiber|5g|quantum",
  "bandwidth_mbps": 100000,
  "latency_ms": 0.5,
  "signal_strength": -50
}
```

#### Layer 2: Data

```json
{
  "format": "JSON-LD",
  "compression": "zstd",
  "size_bytes": 1024,
  "checksum": "sha256:..."
}
```

#### Layer 3: Intelligence

```json
{
  "processing_node": "edge|central",
  "model_id": "traffic-pred-v2",
  "inference_time_ms": 15,
  "confidence": 0.95
}
```

#### Layer 4: Application

```json
{
  "service": "traffic-management",
  "action": "adjust_signal",
  "parameters": {},
  "response_required": true
}
```

---

## 6. Validation Rules

### 6.1 Required Fields
- All messages MUST include `aidc_version`, `layer`, `message_type`, `source`, `payload`
- All transactions MUST include `transaction_id`, `amount`, `contribution`
- All consent tokens MUST include `token_id`, `citizen_id`, `data_type`, `purpose`

### 6.2 ID Format Validation
| ID Type | Pattern | Example |
|---------|---------|---------|
| City ID | `AIC-YYYY-XXXXXX` | AIC-2024-A1B2C3 |
| Transaction ID | `TXN-YYYY-XXXXXXXXXXXX` | TXN-2024-ABC123DEF456 |
| Consent Token | `CST-XXXXXXXXXXXXXXXX` | CST-A1B2C3D4E5F6G7H8 |
| Node ID | `AIDC-NODE-XXX` | AIDC-NODE-001 |

### 6.3 Temporal Rules
- `timestamp` fields MUST be in ISO 8601 format
- `expires_at` MUST be greater than `granted_at` for consent tokens
- All times MUST be in UTC

---

## 7. Examples

### Complete AI-City Instance

```json
{
  "id": "AIC-2024-SEOUL1",
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
    "power_watts": 2000,
    "thermal_target": 25,
    "ai_allocation": 0.7,
    "infra_allocation": 0.2,
    "reserve_allocation": 0.1
  },
  "sharingEngine": {
    "contribution_rate": 0.01,
    "education": 0.003,
    "healthcare": 0.003,
    "environment": 0.002,
    "emergency": 0.002
  },
  "dataSovereignty": {
    "enabled": true,
    "consent_required": true,
    "portability": true,
    "right_to_erasure": true
  },
  "aidcLink": {
    "version": "1.0",
    "layers": 4,
    "encryption": "AES-256-GCM"
  },
  "metadata": {
    "created": "2024-01-01T00:00:00Z",
    "updated": "2024-12-24T12:00:00Z",
    "status": "active"
  }
}
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
