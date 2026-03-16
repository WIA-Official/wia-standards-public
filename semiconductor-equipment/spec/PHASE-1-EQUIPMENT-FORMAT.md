# WIA-SEMI-019 - Phase 1: Equipment Data Format

> **Version:** 1.0  
> **Status:** Active  
> **Last Updated:** 2025-12-26

## 1. Overview

Phase 1 defines the foundational data format for describing semiconductor equipment specifications, capabilities, and parameters. All WIA-SEMI-019 compliant equipment must provide machine-readable specifications in standardized JSON format.

### 1.1 Purpose

- Enable automated equipment discovery and capability assessment
- Provide semantic interoperability across vendors
- Support intelligent equipment selection and scheduling
- Facilitate data-driven analytics and optimization

### 1.2 Scope

Phase 1 covers:
- Equipment specification schema
- Parameter naming conventions
- Data type definitions
- Unit standardization
- Equipment-specific extensions (lithography, etch, deposition, inspection, CMP)

## 2. Equipment Specification Schema

### 2.1 Core Schema Structure

```json
{
  "standard": "WIA-SEMI-019",
  "version": "1.0",
  "last_updated": "2025-12-26T00:00:00Z",
  "equipment": {
    "manufacturer": "string",
    "model": "string",
    "serial_number": "string",
    "type": "string",
    "subtype": "string",
    "installation_date": "ISO8601",
    "wafer_size_mm": [200, 300],
    "process_node_nm": [3, 5, 7],
    "chambers": "integer",
    "load_ports": "integer"
  },
  "capabilities": {},
  "parameters": {},
  "interfaces": {},
  "certification": {}
}
```

### 2.2 Equipment Types

Standardized equipment type values:

- `lithography`: Lithography systems (EUV, DUV, i-line)
- `etch`: Etch equipment (Plasma, RIE, wet etch)
- `deposition`: Deposition tools (CVD, ALD, PVD, epitaxy)
- `inspection`: Inspection and metrology (e-beam, optical, x-ray)
- `cmp`: Chemical mechanical planarization
- `implant`: Ion implantation
- `cleaning`: Wafer cleaning equipment
- `metrology`: Standalone metrology tools
- `anneal`: Thermal processing equipment
- `photolithography`: Coater/developer tracks

### 2.3 Capabilities Object

Equipment-specific capabilities describing performance characteristics:

```json
"capabilities": {
  "technology": "string",
  "throughput_wph": "float",
  "max_wafer_size_mm": "integer",
  "min_feature_size_nm": "float",
  "uniformity_percent": "float",
  "process_temperature_range_celsius": {
    "min": "float",
    "max": "float"
  },
  "materials_supported": ["array of strings"],
  "automation_level": "string"
}
```

## 3. Parameter Dictionary

### 3.1 Naming Conventions

All parameters follow the pattern: `{category}_{parameter}_{unit}`

Rules:
- All lowercase
- Words separated by underscores
- Units always specified
- No abbreviations unless industry-standard (rf, uv, ir)
- Maximum 64 characters

Examples:
- `substrate_temperature_celsius`
- `chamber_pressure_pascal`
- `rf_power_forward_watts`
- `gas_flow_n2_sccm`

### 3.2 Standard Units

| Physical Quantity | Standard Unit | Symbol |
|-------------------|---------------|--------|
| Temperature | Celsius | celsius |
| Pressure | Pascal | pascal |
| Flow Rate | sccm | sccm |
| Power | Watts | watts |
| Voltage | Volts | volts |
| Current | Amperes | amperes |
| Frequency | Hertz | hertz |
| Time | Seconds | seconds |
| Length | Meters | meters |
| Length (small) | Nanometers | nanometers |

### 3.3 Data Types

| Type | Description | Example |
|------|-------------|---------|
| float | Floating-point number | 425.3 |
| integer | Integer number | 100 |
| boolean | True/false | true |
| string | Text string | "WAFER_ID_123" |
| timestamp | ISO 8601 datetime | "2025-12-26T15:30:45.123Z" |
| array | Array of values | [1, 2, 3, 4, 5] |
| object | Nested object | {"key": "value"} |

### 3.4 Parameter Categories

Parameters are organized into categories:

1. **Process Parameters** (4000-4999): Core process control
   - Temperature, pressure, flow, power, time
   - Recipe setpoints and targets

2. **Sensor Parameters** (5000-5999): Measured values
   - Actual temperatures, pressures, flows
   - Real-time measurements

3. **Control Parameters** (6000-6999): Equipment control
   - Valve positions, motor speeds, heater power
   - Hardware control signals

4. **Metrology Parameters** (7000-7999): Measurement results
   - Film thickness, overlay, CD, defect counts
   - Quality metrics

5. **Status Parameters** (8000-8999): Equipment status
   - States, alarms, events, health
   - Operational status

## 4. Equipment-Specific Extensions

### 4.1 Lithography Equipment

```json
{
  "lithography_specific": {
    "exposure": {
      "wavelength_nm": "float",
      "dose_mj_cm2": "float",
      "exposure_time_ms": "float",
      "numerical_aperture": "float",
      "slit_width_mm": "float",
      "scan_speed_mm_s": "float"
    },
    "overlay": {
      "overlay_x_nm": "float",
      "overlay_y_nm": "float",
      "overlay_3sigma_nm": "float",
      "alignment_marks": "integer",
      "alignment_residual_nm": "float"
    },
    "focus": {
      "focus_offset_nm": "float",
      "focus_depth_nm": "float",
      "tilt_x_nm": "float",
      "tilt_y_nm": "float",
      "leveling_residual_nm": "float"
    },
    "reticle": {
      "reticle_id": "string",
      "magnification": "float",
      "pellicle_installed": "boolean",
      "inspection_date": "timestamp"
    }
  }
}
```

### 4.2 Etch Equipment

```json
{
  "etch_specific": {
    "plasma": {
      "rf_power_top_watts": "float",
      "rf_power_bottom_watts": "float",
      "rf_frequency_mhz": "float",
      "dc_bias_volts": "float",
      "plasma_density_cm3": "float"
    },
    "process": {
      "etch_rate_nm_min": "float",
      "selectivity_ratio": "float",
      "uniformity_percent": "float",
      "aspect_ratio_max": "float",
      "sidewall_angle_degrees": "float"
    },
    "gases": {
      "gas_chemistry": "string",
      "total_flow_sccm": "float"
    }
  }
}
```

### 4.3 Deposition Equipment

```json
{
  "deposition_specific": {
    "film": {
      "material": "string",
      "target_thickness_nm": "float",
      "measured_thickness_nm": "float",
      "uniformity_percent": "float",
      "step_coverage_percent": "float",
      "stress_mpa": "float",
      "refractive_index": "float"
    },
    "process": {
      "deposition_rate_nm_min": "float",
      "temperature_celsius": "float",
      "pressure_pascal": "float",
      "method": "string"
    }
  }
}
```

### 4.4 Inspection Equipment

```json
{
  "inspection_specific": {
    "method": "string",
    "resolution_nm": "float",
    "defect_sensitivity_nm": "float",
    "throughput_wph": "float",
    "inspection_area_percent": "float",
    "defects": {
      "total_count": "integer",
      "critical_count": "integer",
      "defect_types": {}
    },
    "classification": {
      "nuisance_percent": "float",
      "systematic_percent": "float",
      "random_percent": "float"
    }
  }
}
```

## 5. Data Quality Indicators

All parameter values include quality metadata:

```json
{
  "parameter": "substrate_temperature_celsius",
  "value": 425.3,
  "timestamp": "2025-12-26T15:30:45.123Z",
  "quality": {
    "status": "GOOD",
    "confidence": 0.98,
    "sensor_health": "NORMAL",
    "calibration_date": "2025-12-01",
    "calibration_due": "2026-03-01",
    "out_of_spec": false,
    "error_code": null
  }
}
```

Quality Status Values:
- `GOOD`: Parameter value is valid and reliable
- `UNCERTAIN`: Parameter may be questionable
- `BAD`: Parameter value is invalid or unreliable
- `UNKNOWN`: Quality cannot be determined

## 6. Validation Rules

### 6.1 Required Fields

All equipment specifications must include:
- `standard`, `version`, `last_updated`
- `equipment.manufacturer`, `equipment.model`, `equipment.type`
- `parameters` object with count of each category
- `interfaces` object indicating supported protocols

### 6.2 Data Range Validation

Parameters must include valid ranges:

```json
{
  "parameter_id": "substrate_temp_celsius",
  "range": {
    "min": 20,
    "max": 500,
    "normal_min": 350,
    "normal_max": 450
  },
  "precision": 0.1,
  "resolution": 0.01
}
```

### 6.3 JSON Schema Validation

Equipment specifications must validate against the WIA-SEMI-019 JSON Schema available at:
`https://standards.wia.org/semi-019/v1.0/equipment-spec.schema.json`

## 7. Implementation Guidelines

### 7.1 Equipment Specification File

Each equipment instance must provide a specification file accessible via:
- HTTP/HTTPS: `http://{equipment-ip}/wia/equipment-spec.json`
- SECS S9F100/S9F101: WIA Equipment Specification Request
- File system: `/wia/equipment-spec.json`

### 7.2 Parameter Discovery

Equipment must support parameter enumeration:
- List all available parameters with metadata
- Indicate which parameters are readable vs. writable
- Provide update frequency for sensor parameters

### 7.3 Custom Parameters

Vendor-specific parameters not in standard dictionary must:
- Follow naming conventions
- Include full metadata (description, unit, range, type)
- Use parameter IDs >=10000 to avoid conflicts
- Document in equipment specification

## 8. Bronze Certification Requirements

To achieve Bronze certification, equipment must:

1. Provide valid equipment specification JSON file
2. Use standardized parameter names for at least 80% of parameters
3. Follow naming conventions for custom parameters
4. Include data quality indicators
5. Validate against JSON schema
6. Document all parameters with complete metadata
7. Support parameter discovery via at least one interface

## 9. Examples

### 9.1 Complete Equipment Specification

```json
{
  "standard": "WIA-SEMI-019",
  "version": "1.0",
  "last_updated": "2025-12-26T10:00:00Z",
  "equipment": {
    "manufacturer": "Applied Materials",
    "model": "Centura 5200",
    "serial_number": "AMAT-2025-001",
    "type": "deposition",
    "subtype": "pecvd",
    "installation_date": "2025-01-15",
    "wafer_size_mm": [200, 300],
    "process_node_nm": [5, 7, 10, 14],
    "chambers": 4,
    "load_ports": 2
  },
  "capabilities": {
    "technology": "PECVD",
    "throughput_wph": 100,
    "materials_supported": ["SiO2", "SiN", "SiON"],
    "max_temperature_celsius": 450,
    "uniformity_percent": 1.5,
    "step_coverage_percent": 95
  },
  "parameters": {
    "process": 127,
    "sensor": 438,
    "control": 156,
    "metrology": 45,
    "status": 89
  },
  "interfaces": {
    "secs_gem": true,
    "rest_api": true,
    "websocket": true,
    "e84": true
  },
  "certification": {
    "wia_level": "Silver",
    "certified_date": "2025-11-10",
    "certificate_id": "WIA-SEMI-019-AMAT-001",
    "valid_until": "2026-11-10"
  }
}
```

---

**弘益人間 · Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 MIT License*
