# WIA-SEMI-001: Phase 1 - Data Format Specification

Version: 1.0
Status: Final
Date: 2025-01-15

## Overview

Phase 1 of WIA-SEMI-001 establishes standardized data formats for semiconductor specifications, enabling machine-readable, consistent documentation across the industry. This phase forms the foundation for all subsequent phases.

## Design Principles

1. **Machine Readability**: All data must be parseable by automated tools
2. **Human Friendliness**: Formats should be editable by engineers
3. **Validation**: Strict schemas prevent invalid data
4. **Completeness**: Capture all necessary information
5. **Versioning**: Clear version identification and evolution

## Core Data Format

All WIA-SEMI-001 specifications use JSON format with strict schema validation.

### Root Schema Structure

```json
{
  "$schema": "https://wia.org/schemas/semi-001/v1/chip-spec.json",
  "wiaVersion": "1.0",
  "metadata": { ... },
  "architecture": { ... },
  "power": { ... },
  "thermal": { ... },
  "interfaces": { ... },
  "performance": { ... }
}
```

## Metadata Specification

### Required Fields

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| id | string | Globally unique identifier | "WIA-SOC-2025-001" |
| manufacturer | object | Manufacturer information | See below |
| productName | string | Commercial product name | "ExampleSoC Gen 5" |
| productFamily | string | Product family | "ExampleSoC Gen 5" |
| marketSegment | array | Target markets | ["mobile", "tablet"] |
| releaseDate | ISO 8601 | Release date | "2025-03-15" |
| productionStatus | enum | Lifecycle status | "active" |

### Manufacturer Object

```json
{
  "manufacturer": {
    "name": "Example Semiconductor Corp",
    "division": "Mobile Solutions",
    "contact": "support@example.com",
    "website": "https://example.com"
  }
}
```

### Production Status Values

- `announced`: Product announced but not sampling
- `sampling`: Engineering samples available
- `active`: In mass production
- `eol-announced`: End-of-life announced
- `eol`: No longer manufactured

## Architecture Specification

### CPU Architecture

```json
{
  "cpu": {
    "clusters": [
      {
        "name": "Performance",
        "coreCount": 2,
        "microarchitecture": "Cortex-X5",
        "frequency": {
          "min": 1.0,
          "max": 3.8,
          "unit": "GHz"
        },
        "cache": {
          "l1i": {"size": 64, "unit": "KB"},
          "l1d": {"size": 64, "unit": "KB"},
          "l2": {"size": 2, "unit": "MB"}
        },
        "features": ["out-of-order", "SMT", "vector-extensions"]
      }
    ],
    "sharedCache": {
      "l3": {"size": 16, "unit": "MB"}
    }
  }
}
```

### GPU Architecture

```json
{
  "gpu": {
    "architecture": "Immortalis-G925",
    "computeUnits": 16,
    "frequency": {
      "min": 400,
      "max": 1150,
      "unit": "MHz"
    },
    "features": ["ray-tracing", "variable-rate-shading"],
    "apis": ["Vulkan 1.3", "OpenGL ES 3.2"]
  }
}
```

### NPU Architecture

```json
{
  "npu": {
    "architecture": "Custom-AI-v5",
    "performance": {
      "value": 50,
      "unit": "TOPS"
    },
    "precision": ["INT4", "INT8", "FP16", "BF16"],
    "frameworks": ["TensorFlow Lite", "PyTorch Mobile"]
  }
}
```

## Power Specification

### TDP and Power Rails

```json
{
  "power": {
    "tdp": {
      "typical": {"value": 8.5, "unit": "W"},
      "maximum": {"value": 12.0, "unit": "W"}
    },
    "voltageRails": [
      {
        "name": "VDD_CPU",
        "nominal": 0.85,
        "min": 0.65,
        "max": 1.05,
        "unit": "V",
        "tolerance": 0.05
      }
    ],
    "powerStates": [
      {
        "name": "Active-Max",
        "cpuFrequency": 3.8,
        "gpuFrequency": 1150,
        "power": {"typical": 12.0, "unit": "W"}
      },
      {
        "name": "Idle",
        "cpuFrequency": 0.5,
        "gpuFrequency": 400,
        "power": {"typical": 0.8, "unit": "W"}
      }
    ]
  }
}
```

### Power Management Features

- **DVFS**: Dynamic Voltage and Frequency Scaling
- **Power Gating**: Turning off unused components
- **Clock Gating**: Stopping clocks to idle components
- **Adaptive Voltage**: Adjusting voltage based on process variation

## Thermal Specification

```json
{
  "thermal": {
    "junctionTemperature": {
      "typical": 75,
      "maximum": 105,
      "unit": "°C"
    },
    "thermalResistance": {
      "junctionToCase": {"value": 0.8, "unit": "°C/W"}
    },
    "hotspots": [
      {
        "location": "CPU-Performance-Cluster",
        "peakTemperature": 95,
        "area": {"value": 8.5, "unit": "mm²"}
      }
    ],
    "thermalThrottling": {
      "enabled": true,
      "thresholds": [
        {
          "temperature": 95,
          "action": "reduce-frequency-25%",
          "unit": "°C"
        }
      ]
    }
  }
}
```

## Performance Metrics

### Standard Benchmarks

All performance claims must specify benchmark, version, and test conditions:

```json
{
  "performance": {
    "cpu": {
      "singleCore": {
        "benchmark": "Geekbench 6.2",
        "score": 2850,
        "testConditions": "Max frequency, typical power"
      },
      "multiCore": {
        "benchmark": "Geekbench 6.2",
        "score": 9200,
        "testConditions": "All cores active, sustained"
      }
    },
    "gpu": {
      "graphics": {
        "benchmark": "GFXBench 5.0 Manhattan 3.1.1",
        "offscreen": {"value": 245, "unit": "fps"},
        "conditions": {
          "resolution": "1080p",
          "api": "Vulkan 1.3"
        }
      }
    }
  }
}
```

## Type Definitions

### Frequency Type

```json
{
  "type": "object",
  "properties": {
    "value": {"type": "number", "minimum": 0},
    "unit": {"type": "string", "enum": ["Hz", "kHz", "MHz", "GHz"]}
  },
  "required": ["value", "unit"]
}
```

### Power Type

```json
{
  "type": "object",
  "properties": {
    "value": {"type": "number", "minimum": 0},
    "unit": {"type": "string", "enum": ["mW", "W"]}
  },
  "required": ["value", "unit"]
}
```

### Temperature Type

```json
{
  "type": "object",
  "properties": {
    "value": {"type": "number"},
    "unit": {"type": "string", "enum": ["°C", "°F", "K"]}
  },
  "required": ["value", "unit"]
}
```

## Validation

All specifications must pass JSON Schema validation:

```bash
wia-validator validate --schema chip-spec-v1.json --input my-chip.json
```

### Validation Rules

1. All required fields must be present
2. Values must match specified types
3. Enumerations must use approved values
4. Units must be specified for all measurements
5. Cross-field validation (e.g., min < max)

## Extensibility

Vendors can add custom fields using the `x-` prefix:

```json
{
  "x-vendor-specific": {
    "customFeature": "value"
  }
}
```

Extensions must not conflict with standard fields and should be documented.

## Version Management

- Specifications include `wiaVersion` field
- Semantic versioning: MAJOR.MINOR.PATCH
- Breaking changes increment MAJOR version
- New features increment MINOR version
- Bug fixes increment PATCH version

## Compliance Requirements

To be Phase 1 compliant:

1. ✓ All chip specifications in WIA JSON format
2. ✓ Pass schema validation without errors
3. ✓ Include all required fields with accurate data
4. ✓ Use standard units and enumerations
5. ✓ Maintain specification version control

## Tools and Resources

- **Schema Files**: https://wia.org/schemas/semi-001/
- **Validator**: `npm install -g wia-semi-validator`
- **Examples**: https://github.com/WIA-Official/wia-standards/examples
- **Documentation**: https://docs.wia.org/semi-001/phase-1

## Migration Guide

### From PDF Documentation

1. Extract structured data from PDF specifications
2. Map fields to WIA schema
3. Validate against schema
4. Fill in any missing required fields
5. Publish WIA-compliant JSON

### From Proprietary Formats

1. Write conversion script using WIA SDK
2. Validate output
3. Review and correct automated conversion
4. Establish ongoing conversion process

## Benefits of Phase 1 Adoption

- **Immediate**: Better data exchange with customers
- **Automated Tools**: Enable specification parsing and analysis
- **Comparisons**: Facilitate accurate cross-vendor comparisons
- **Low Cost**: Minimal implementation cost, high value

---

**Next**: [Phase 2 - API Interface Standards](PHASE-2-API-INTERFACE.md)

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
弘益人間 (Hongik Ingan) - Benefit All Humanity
