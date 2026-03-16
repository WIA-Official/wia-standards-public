# WIA-FUSION Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the standardized data formats for nuclear fusion energy systems, enabling interoperability between different reactor types (Tokamak, Stellarator, Laser Fusion) and research institutions worldwide.

### 1.1 Scope

- Plasma state representation
- Energy output metrics
- Diagnostic data formats
- Control system parameters
- Safety monitoring data

### 1.2 Design Principles

1. **Technology Agnostic**: Support all fusion approaches
2. **Real-time Capable**: Sub-millisecond data updates
3. **Extensible**: Custom fields for experimental data
4. **Interoperable**: Compatible with ITER, KSTAR, JET systems

---

## 2. Plasma State Schema

### 2.1 Core Schema Definition

```json
{
  "$schema": "https://wia.live/schemas/fusion/v1.0.0",
  "type": "object",
  "required": ["plasma_state"],
  "properties": {
    "plasma_state": {
      "type": "object",
      "required": ["shot_id", "timestamp", "reactor", "core_parameters"],
      "properties": {
        "shot_id": {
          "type": "string",
          "pattern": "^[A-Z]+-[0-9]+$",
          "description": "Unique identifier for plasma shot"
        },
        "timestamp": {
          "type": "string",
          "format": "date-time",
          "description": "ISO8601 timestamp"
        },
        "reactor": {
          "type": "string",
          "enum": ["ITER", "KSTAR", "JET", "EAST", "SPARC", "W7X", "NIF", "custom"],
          "description": "Reactor identification"
        },
        "core_parameters": {
          "$ref": "#/definitions/CoreParameters"
        },
        "performance": {
          "$ref": "#/definitions/Performance"
        },
        "stability": {
          "$ref": "#/definitions/Stability"
        },
        "control": {
          "$ref": "#/definitions/Control"
        }
      }
    }
  }
}
```

### 2.2 Core Parameters

```json
{
  "definitions": {
    "CoreParameters": {
      "type": "object",
      "properties": {
        "temperature_keV": {
          "type": "object",
          "properties": {
            "ion": { "type": "number", "minimum": 0, "maximum": 100 },
            "electron": { "type": "number", "minimum": 0, "maximum": 100 }
          },
          "description": "Plasma temperature in keV (1 keV ≈ 11.6 million °C)"
        },
        "density_m3": {
          "type": "object",
          "properties": {
            "value": { "type": "number", "minimum": 0 },
            "unit": { "type": "string", "const": "1e20/m3" }
          },
          "description": "Plasma density in units of 10²⁰ particles per cubic meter"
        },
        "confinement_time_s": {
          "type": "number",
          "minimum": 0,
          "description": "Energy confinement time in seconds"
        },
        "triple_product": {
          "type": "object",
          "properties": {
            "value": { "type": "number" },
            "unit": { "type": "string", "const": "keV·s·1e20/m3" }
          },
          "description": "Fusion triple product (n × T × τ)"
        }
      }
    }
  }
}
```

### 2.3 Performance Metrics

```json
{
  "definitions": {
    "Performance": {
      "type": "object",
      "properties": {
        "q_factor": {
          "type": "number",
          "minimum": 0,
          "description": "Fusion gain factor (P_fusion / P_input)"
        },
        "fusion_power_mw": {
          "type": "number",
          "minimum": 0,
          "description": "Total fusion power output in megawatts"
        },
        "plasma_current_ma": {
          "type": "number",
          "description": "Plasma current in mega-amperes"
        },
        "beta_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 10,
          "description": "Ratio of plasma pressure to magnetic pressure"
        },
        "bootstrap_fraction": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Self-generated current fraction"
        }
      }
    }
  }
}
```

### 2.4 Stability Parameters

```json
{
  "definitions": {
    "Stability": {
      "type": "object",
      "properties": {
        "disruption_risk": {
          "type": "object",
          "properties": {
            "value": { "type": "number", "minimum": 0, "maximum": 1 },
            "confidence": { "type": "number", "minimum": 0, "maximum": 1 }
          },
          "description": "AI-predicted disruption probability"
        },
        "elm_frequency_hz": {
          "type": "number",
          "minimum": 0,
          "description": "Edge Localized Mode frequency"
        },
        "mhd_activity": {
          "type": "array",
          "items": {
            "type": "string",
            "pattern": "^m=[0-9]+,n=[0-9]+$"
          },
          "description": "Active MHD modes (m,n notation)"
        },
        "locked_mode": {
          "type": "boolean",
          "description": "Locked mode detection"
        },
        "vertical_displacement": {
          "type": "number",
          "description": "Vertical displacement event indicator"
        }
      }
    }
  }
}
```

### 2.5 Control Parameters

```json
{
  "definitions": {
    "Control": {
      "type": "object",
      "properties": {
        "heating_power_mw": {
          "type": "object",
          "properties": {
            "ohmic": { "type": "number", "minimum": 0 },
            "nbi": { "type": "number", "minimum": 0, "description": "Neutral Beam Injection" },
            "icrh": { "type": "number", "minimum": 0, "description": "Ion Cyclotron Resonance Heating" },
            "ecrh": { "type": "number", "minimum": 0, "description": "Electron Cyclotron Resonance Heating" },
            "lhcd": { "type": "number", "minimum": 0, "description": "Lower Hybrid Current Drive" }
          }
        },
        "magnetic_field_t": {
          "type": "number",
          "description": "Toroidal magnetic field in Tesla"
        },
        "divertor_heat_mw_m2": {
          "type": "number",
          "description": "Divertor heat flux in MW/m²"
        },
        "fuel_mix": {
          "type": "object",
          "properties": {
            "deuterium": { "type": "number", "minimum": 0, "maximum": 1 },
            "tritium": { "type": "number", "minimum": 0, "maximum": 1 }
          }
        }
      }
    }
  }
}
```

---

## 3. Energy Output Schema

### 3.1 Power Plant Metrics

```json
{
  "energy_output": {
    "type": "object",
    "properties": {
      "gross_fusion_power_mw": {
        "type": "number",
        "description": "Total fusion power generated"
      },
      "thermal_power_mw": {
        "type": "number",
        "description": "Thermal power to heat exchangers"
      },
      "net_electric_power_mw": {
        "type": "number",
        "description": "Net electrical output after auxiliaries"
      },
      "plant_efficiency_percent": {
        "type": "number",
        "minimum": 0,
        "maximum": 100
      },
      "availability_factor": {
        "type": "number",
        "minimum": 0,
        "maximum": 1
      },
      "tritium_breeding_ratio": {
        "type": "number",
        "description": "TBR ≥ 1.05 for self-sufficiency"
      },
      "neutron_wall_loading": {
        "type": "object",
        "properties": {
          "value": { "type": "number" },
          "unit": { "type": "string", "const": "MW/m2" }
        }
      }
    }
  }
}
```

---

## 4. Diagnostic Data Format

### 4.1 Standard Diagnostics

| Diagnostic | Data Type | Sampling Rate | Description |
|------------|-----------|---------------|-------------|
| Thomson Scattering | Array[float] | 100 Hz | Electron temperature/density profile |
| Interferometry | float | 10 kHz | Line-integrated density |
| ECE | Array[float] | 1 MHz | Electron temperature profile |
| Bolometry | Array[float] | 10 kHz | Radiated power distribution |
| Magnetics | Array[float] | 100 kHz | Magnetic field measurements |
| Neutron Counter | int | 1 kHz | Fusion reaction rate |
| Soft X-ray | Array[float] | 100 kHz | Core impurity monitoring |

### 4.2 Diagnostic Data Schema

```json
{
  "diagnostic_data": {
    "type": "object",
    "properties": {
      "diagnostic_id": { "type": "string" },
      "timestamp": { "type": "string", "format": "date-time" },
      "sampling_rate_hz": { "type": "number" },
      "data": {
        "type": "array",
        "items": { "type": "number" }
      },
      "units": { "type": "string" },
      "calibration_version": { "type": "string" },
      "quality_flag": {
        "type": "string",
        "enum": ["good", "suspect", "bad", "missing"]
      }
    }
  }
}
```

---

## 5. Data Exchange Protocols

### 5.1 Real-time Streaming

```yaml
protocol: WebSocket
endpoint: wss://fusion.wia.live/v1/stream
format: JSON
compression: gzip
max_latency_ms: 10
authentication: JWT Bearer Token
```

### 5.2 Batch Data Transfer

```yaml
protocol: HTTPS
endpoint: https://fusion.wia.live/v1/shots/{shot_id}
format: HDF5 / NetCDF / JSON
max_size_gb: 100
authentication: API Key + Signature
```

---

## 6. Compliance Requirements

### 6.1 Mandatory Fields

All implementations MUST include:
- `shot_id`
- `timestamp`
- `reactor`
- `core_parameters.temperature_keV`
- `core_parameters.density_m3`

### 6.2 Data Validation

```python
# Example validation
def validate_plasma_state(data: dict) -> bool:
    required = ['shot_id', 'timestamp', 'reactor', 'core_parameters']
    return all(field in data.get('plasma_state', {}) for field in required)
```

---

## 7. References

- ITER Organization Data Standards
- IAEA Nuclear Fusion Data Guidelines
- KSTAR Data Management System
- IMAS (Integrated Modelling & Analysis Suite)

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association
