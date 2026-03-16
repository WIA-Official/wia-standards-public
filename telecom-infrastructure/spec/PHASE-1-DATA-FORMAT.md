# WIA-SOC-012 Telecommunications Infrastructure Standard
## Phase 1: Data Format Specification

> **Version**: 1.0.0  
> **Status**: Stable  
> **Last Updated**: 2025

---

## 1. Overview

Phase 1 defines the standardized data formats for representing telecommunications infrastructure information. This specification ensures interoperability across platforms, vendors, and applications by establishing a common language for infrastructure data exchange.

### 1.1 Design Principles

- **Interoperability**: Universal format compatible with any telecommunications system
- **Extensibility**: Support for future technologies and standards
- **Precision**: Accurate representation of infrastructure parameters
- **Completeness**: Comprehensive coverage of all infrastructure types

---

## 2. Core Schema

### 2.1 Infrastructure Root Schema

\`\`\`json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wiastandards.com/schemas/telecom-infra/v1/infrastructure.json",
  "title": "WIA Telecommunications Infrastructure",
  "description": "Standardized telecommunications infrastructure data format",
  "type": "object",
  "required": ["infra_id", "timestamp", "version", "type"],
  "properties": {
    "infra_id": {
      "type": "string",
      "format": "uuid",
      "description": "Unique identifier for infrastructure element"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp of data collection"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Schema version (SemVer)"
    },
    "type": {
      "type": "string",
      "enum": ["cell_tower", "fiber_node", "data_center", "edge_node", "backhaul"],
      "description": "Infrastructure element type"
    },
    "location": {
      "$ref": "#/$defs/Location"
    },
    "specifications": {
      "$ref": "#/$defs/Specifications"
    },
    "telemetry": {
      "$ref": "#/$defs/Telemetry"
    },
    "metadata": {
      "$ref": "#/$defs/Metadata"
    }
  }
}
\`\`\`

### 2.2 Location Schema

\`\`\`json
{
  "$defs": {
    "Location": {
      "type": "object",
      "required": ["latitude", "longitude"],
      "properties": {
        "latitude": {
          "type": "number",
          "minimum": -90,
          "maximum": 90,
          "description": "Latitude in decimal degrees"
        },
        "longitude": {
          "type": "number",
          "minimum": -180,
          "maximum": 180,
          "description": "Longitude in decimal degrees"
        },
        "altitude": {
          "type": "number",
          "description": "Altitude in meters above sea level"
        },
        "address": {
          "type": "string",
          "description": "Physical address"
        },
        "country_code": {
          "type": "string",
          "pattern": "^[A-Z]{2}$",
          "description": "ISO 3166-1 alpha-2 country code"
        }
      }
    }
  }
}
\`\`\`

### 2.3 Cell Tower Specifications

\`\`\`json
{
  "$defs": {
    "CellTowerSpec": {
      "type": "object",
      "required": ["tower_type", "height", "sectors"],
      "properties": {
        "tower_type": {
          "type": "string",
          "enum": ["macro", "micro", "small_cell", "das"],
          "description": "Type of cell site"
        },
        "height": {
          "type": "number",
          "description": "Tower height in meters",
          "minimum": 0
        },
        "sectors": {
          "type": "array",
          "items": {
            "$ref": "#/$defs/Sector"
          },
          "minItems": 1,
          "maxItems": 12
        },
        "backhaul": {
          "$ref": "#/$defs/Backhaul"
        },
        "power": {
          "$ref": "#/$defs/PowerSystem"
        }
      }
    },
    "Sector": {
      "type": "object",
      "required": ["azimuth", "frequencies", "technology"],
      "properties": {
        "azimuth": {
          "type": "number",
          "minimum": 0,
          "maximum": 360,
          "description": "Antenna azimuth in degrees"
        },
        "tilt": {
          "type": "object",
          "properties": {
            "mechanical": {
              "type": "number",
              "minimum": -10,
              "maximum": 10
            },
            "electrical": {
              "type": "number",
              "minimum": -10,
              "maximum": 10
            }
          }
        },
        "frequencies": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "band": {
                "type": "string",
                "description": "Frequency band (e.g., 'n78', 'B2')"
              },
              "frequency": {
                "type": "number",
                "description": "Center frequency in MHz"
              },
              "bandwidth": {
                "type": "number",
                "description": "Channel bandwidth in MHz"
              }
            }
          }
        },
        "technology": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["2G", "3G", "4G", "5G", "6G"]
          }
        },
        "antenna": {
          "type": "object",
          "properties": {
            "model": {"type": "string"},
            "gain": {"type": "number", "description": "Antenna gain in dBi"},
            "mimo_config": {"type": "string", "description": "MIMO configuration (e.g., '64T64R')"}
          }
        }
      }
    }
  }
}
\`\`\`

### 2.4 Fiber Infrastructure Specifications

\`\`\`json
{
  "$defs": {
    "FiberSpec": {
      "type": "object",
      "required": ["fiber_type", "fiber_count"],
      "properties": {
        "fiber_type": {
          "type": "string",
          "enum": ["single_mode", "multi_mode", "specialty"],
          "description": "Type of fiber optic cable"
        },
        "fiber_count": {
          "type": "integer",
          "minimum": 1,
          "description": "Number of fibers in cable"
        },
        "cable_length": {
          "type": "number",
          "description": "Cable length in kilometers"
        },
        "wavelengths": {
          "type": "array",
          "items": {
            "type": "number",
            "description": "Wavelength in nanometers (e.g., 1310, 1550)"
          }
        },
        "dwdm_channels": {
          "type": "integer",
          "description": "Number of DWDM channels (if applicable)"
        },
        "capacity": {
          "type": "number",
          "description": "Total capacity in Gbps"
        },
        "installation_method": {
          "type": "string",
          "enum": ["aerial", "underground_conduit", "direct_burial", "submarine"],
          "description": "Cable installation method"
        }
      }
    }
  }
}
\`\`\`

### 2.5 Telemetry Data

\`\`\`json
{
  "$defs": {
    "Telemetry": {
      "type": "object",
      "properties": {
        "performance": {
          "type": "object",
          "properties": {
            "throughput_mbps": {"type": "number"},
            "latency_ms": {"type": "number"},
            "packet_loss_percent": {"type": "number"},
            "active_users": {"type": "integer"}
          }
        },
        "power": {
          "type": "object",
          "properties": {
            "consumption_watts": {"type": "number"},
            "battery_level_percent": {"type": "number"},
            "generator_status": {
              "type": "string",
              "enum": ["off", "standby", "running"]
            }
          }
        },
        "environmental": {
          "type": "object",
          "properties": {
            "temperature_celsius": {"type": "number"},
            "humidity_percent": {"type": "number"},
            "wind_speed_mps": {"type": "number"}
          }
        },
        "signal_quality": {
          "type": "object",
          "properties": {
            "rsrp_dbm": {"type": "number", "description": "Reference Signal Received Power"},
            "rsrq_db": {"type": "number", "description": "Reference Signal Received Quality"},
            "sinr_db": {"type": "number", "description": "Signal-to-Interference-plus-Noise Ratio"}
          }
        }
      }
    }
  }
}
\`\`\`

---

## 3. Network Topology Format

### 3.1 Topology Graph

\`\`\`json
{
  "topology": {
    "nodes": [
      {
        "node_id": "uuid",
        "type": "cell_tower | fiber_node | edge_node",
        "location": {"latitude": 0, "longitude": 0}
      }
    ],
    "links": [
      {
        "link_id": "uuid",
        "source_node": "uuid",
        "target_node": "uuid",
        "link_type": "fiber | microwave | satellite",
        "capacity_gbps": 100,
        "latency_ms": 1.5
      }
    ]
  }
}
\`\`\`

---

## 4. Spectrum Allocation Format

\`\`\`json
{
  "spectrum_allocation": {
    "operator_id": "uuid",
    "allocations": [
      {
        "frequency_band": "3.5 GHz",
        "start_frequency_mhz": 3400,
        "end_frequency_mhz": 3500,
        "bandwidth_mhz": 100,
        "license_type": "exclusive | shared | unlicensed",
        "expiration_date": "2030-12-31",
        "coverage_area": {
          "type": "Polygon",
          "coordinates": []
        }
      }
    ]
  }
}
\`\`\`

---

## 5. Validation Rules

### 5.1 Required Fields
All infrastructure elements must include:
- Unique identifier (UUID v4)
- Timestamp (ISO 8601)
- Schema version
- Type designation
- Geographic location

### 5.2 Data Quality
- Coordinates: Must be valid WGS84 lat/lon
- Timestamps: UTC timezone required
- Measurements: SI units mandatory
- Frequencies: MHz for consistency

### 5.3 Performance Constraints
- JSON payload size: Max 1 MB per element
- Nested depth: Max 10 levels
- Array size: Max 1000 items

---

## 6. Example: Complete Cell Tower Record

\`\`\`json
{
  "infra_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-15T14:30:00Z",
  "version": "1.0.0",
  "type": "cell_tower",
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude": 50,
    "address": "123 Market St, San Francisco, CA 94103",
    "country_code": "US"
  },
  "specifications": {
    "tower_type": "macro",
    "height": 45,
    "sectors": [
      {
        "azimuth": 0,
        "tilt": {"mechanical": -2, "electrical": -4},
        "frequencies": [
          {"band": "n78", "frequency": 3500, "bandwidth": 100}
        ],
        "technology": ["4G", "5G"],
        "antenna": {
          "model": "Ericsson AIR 6488",
          "gain": 21,
          "mimo_config": "64T64R"
        }
      }
    ],
    "backhaul": {
      "type": "fiber",
      "capacity_gbps": 100
    },
    "power": {
      "type": "grid",
      "backup": "battery_generator",
      "backup_duration_hours": 24
    }
  },
  "telemetry": {
    "performance": {
      "throughput_mbps": 2500,
      "latency_ms": 10,
      "packet_loss_percent": 0.01,
      "active_users": 1250
    },
    "power": {
      "consumption_watts": 3500,
      "battery_level_percent": 100
    },
    "environmental": {
      "temperature_celsius": 22,
      "humidity_percent": 45
    },
    "signal_quality": {
      "rsrp_dbm": -80,
      "rsrq_db": -10,
      "sinr_db": 20
    }
  },
  "metadata": {
    "owner": "Operator Inc.",
    "deployment_date": "2024-03-15",
    "last_maintenance": "2024-12-01",
    "status": "operational"
  }
}
\`\`\`

---

## 7. Compliance

Implementations MUST:
- Validate all JSON against provided schemas
- Reject malformed or invalid data
- Log validation errors with details
- Support schema versioning

Implementations SHOULD:
- Compress large payloads (gzip, brotli)
- Cache frequently accessed data
- Implement incremental updates

---

**WIA-SOC-012 Phase 1 v1.0**  
© 2025 SmileStory Inc. / WIA
