# WIA 3D Printing Construction Standard
## Phase 1: Data Format Specification
### Version 1.0.0

---

## 1. Introduction

This specification defines standardized data formats for 3D printing construction projects. Phase 1 establishes JSON schemas for building models, material specifications, layer definitions, structural metadata, quality requirements, and process parameters.

### 1.1 Philosophy

Following 弘益人間 (Benefit All Humanity), this specification is:
- Freely accessible without licensing fees
- Vendor-neutral and platform-independent
- Designed for global applicability
- Open to community contribution

### 1.2 Scope

Phase 1 covers:
- Project metadata schemas
- Geometric representations (mesh, parametric, layer-based)
- Material property definitions
- Print parameter specifications
- Quality requirements
- Reinforcement integration
- As-built documentation

## 2. JSON Schema Foundation

All WIA data formats use JSON (JavaScript Object Notation) with JSON Schema validation.

### 2.1 Design Rationale

JSON provides:
- Human readability for debugging and manual editing
- Universal parser support across programming languages
- Compact text-based representation
- Strong schema validation capabilities
- Widespread adoption in web APIs and data exchange

### 2.2 Schema Validation

Every WIA JSON file MUST validate against official JSON Schema definitions available at:
```
https://standards.wia.org/schemas/3d-printing-construction/v1.0/
```

### 2.3 Base Schema Structure

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA 3D Printing Construction Project",
  "type": "object",
  "required": ["standard", "version", "project", "geometry"],
  "properties": {
    "standard": {
      "type": "string",
      "const": "WIA-3D-PRINTING-CONSTRUCTION"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Semantic version (major.minor.patch)"
    },
    "project": { "$ref": "#/definitions/Project" },
    "geometry": { "$ref": "#/definitions/Geometry" },
    "materials": { "$ref": "#/definitions/Materials" },
    "printParameters": { "$ref": "#/definitions/PrintParameters" },
    "quality": { "$ref": "#/definitions/Quality" }
  }
}
```

## 3. Project Metadata

Project metadata identifies and describes the construction project.

### 3.1 Required Fields

| Field | Type | Description |
|-------|------|-------------|
| id | UUID v4 | Unique project identifier |
| name | String | Human-readable project name |
| type | Enum | Building classification |
| location | Object | Geographic location |
| dates | Object | Timeline milestones |

### 3.2 Building Types

Valid building type values:
- `residential` - Single/multi-family housing
- `commercial` - Retail, office, hospitality
- `industrial` - Warehouses, manufacturing
- `infrastructure` - Bridges, utilities, public works
- `institutional` - Schools, hospitals, government
- `mixed-use` - Multiple classifications

### 3.3 Location Schema

```json
{
  "location": {
    "address": "String (optional)",
    "city": "String (required)",
    "state": "String (optional)",
    "country": "ISO 3166-1 alpha-3 (required)",
    "coordinates": {
      "latitude": "Number [-90, 90]",
      "longitude": "Number [-180, 180]",
      "elevation": "Number (meters above sea level)"
    }
  }
}
```

## 4. Geometry Specification

Three geometric representations supported: mesh, parametric, and layer-based.

### 4.1 Mesh Representation

Triangle mesh format for complex geometries:

```json
{
  "geometry": {
    "format": "mesh",
    "meshData": {
      "vertices": [[x, y, z], ...],
      "faces": [[v1, v2, v3], ...],
      "normals": [[nx, ny, nz], ...],
      "unit": "mm"
    },
    "boundingBox": {
      "min": [x, y, z],
      "max": [x, y, z]
    }
  }
}
```

**Validation Requirements:**
- All vertices referenced by faces MUST exist
- Mesh MUST be manifold (closed, no holes)
- Normals MUST be unit vectors
- Bounding box MUST encompass all vertices

### 4.2 Parametric Representation

Simplified representation for regular geometries:

```json
{
  "geometry": {
    "format": "parametric",
    "components": [
      {
        "type": "wall | slab | column | beam",
        "path": [[x, y], ...],
        "height": "Number (mm)",
        "thickness": "Number (mm)",
        "material": "Material ID reference"
      }
    ],
    "unit": "mm"
  }
}
```

### 4.3 Layer-Based Representation

Direct print paths for immediate execution:

```json
{
  "geometry": {
    "format": "layers",
    "layerHeight": "Number (mm)",
    "layers": [
      {
        "number": "Integer (1-indexed)",
        "height": "Number (mm from base)",
        "paths": [
          {
            "type": "perimeter | infill | support",
            "points": [[x, y], ...],
            "width": "Number (mm)",
            "density": "Number [0, 1]"
          }
        ]
      }
    ]
  }
}
```

## 5. Material Specifications

Comprehensive material definitions include printability and structural properties.

### 5.1 Material Library Structure

```json
{
  "materials": {
    "primary": {
      "id": "WIA-{CATEGORY}-{TYPE}-{VARIANT}",
      "name": "String",
      "category": "concrete | polymer | metal | composite | earth",
      "composition": {
        "components": [
          {
            "material": "String",
            "proportion": "Number [0, 1]",
            "specification": "String"
          }
        ]
      },
      "properties": {
        "printability": { /* See 5.2 */ },
        "structural": { /* See 5.3 */ },
        "durability": { /* See 5.4 */ },
        "thermal": { /* See 5.5 */ }
      }
    }
  }
}
```

### 5.2 Printability Properties

| Property | Unit | Description | Test Method |
|----------|------|-------------|-------------|
| flowRate | mm³/s | Material flow through nozzle | Rheometer |
| extrusionPressure | MPa | Required pump pressure | In-line sensor |
| buildability | kPa | Green strength to support layers | Penetrometer |
| openTime | minutes | Workable period | Slump flow over time |

### 5.3 Structural Properties

| Property | Unit | Description | Test Method |
|----------|------|-------------|-------------|
| compressiveStrength | MPa | 28-day compressive strength | ASTM C39 |
| tensileStrength | MPa | Direct tensile strength | ASTM C496 |
| modulusOfElasticity | MPa | Elastic modulus | ASTM C469 |
| flexuralStrength | MPa | Bending strength | ASTM C78 |
| bondStrength | MPa | Inter-layer adhesion | Pull-off test |

### 5.4 Durability Properties

| Property | Values | Test Method |
|----------|--------|-------------|
| freezeThaw | pass/fail | ASTM C666 |
| sulfateResistance | low/medium/high | ASTM C1012 |
| carbonation | low/medium/high | Accelerated carbonation |
| permeability | m/s | ASTM D5084 |

### 5.5 Thermal Properties

| Property | Unit | Test Method |
|----------|------|-------------|
| conductivity | W/(m·K) | ASTM C518 |
| expansion | μm/(m·°C) | ASTM E831 |
| specificHeat | J/(kg·K) | ASTM C351 |

## 6. Print Parameters

Process control parameters define how material is deposited.

### 6.1 Core Parameters

```json
{
  "printParameters": {
    "layerHeight": "Number (mm) [5, 100]",
    "printSpeed": {
      "perimeter": "Number (mm/s) [10, 300]",
      "infill": "Number (mm/s) [10, 500]",
      "unit": "mm/s"
    },
    "pathWidth": {
      "outer": "Number (mm) [20, 100]",
      "inner": "Number (mm) [20, 100]",
      "unit": "mm"
    },
    "temperature": {
      "material": "Number (°C)",
      "ambient": "Number (°C)",
      "unit": "celsius"
    },
    "flowRate": {
      "target": "Number (mm³/s)",
      "tolerance": "Number (mm³/s)",
      "unit": "mm³/s"
    },
    "acceleration": {
      "max": "Number (mm/s²)",
      "unit": "mm/s²"
    }
  }
}
```

### 6.2 Parameter Constraints

Implementations MUST validate parameters against material specifications and equipment capabilities.

**Validation Rules:**
1. `flowRate.target` MUST be within material's `flowRate` range
2. `printSpeed * pathWidth * layerHeight` MUST equal `flowRate` (within tolerance)
3. `layerHeight` MUST support adequate `buildability` for target height
4. `temperature` ranges MUST maintain material `openTime`

## 7. Quality Requirements

Quality specifications define acceptance criteria.

### 7.1 Dimensional Tolerances

```json
{
  "quality": {
    "dimensional": {
      "tolerances": {
        "horizontal": {
          "value": "Number (mm)",
          "method": "laser-measurement | manual"
        },
        "vertical": {
          "value": "Number (mm)",
          "method": "laser-measurement | manual"
        },
        "wallThickness": {
          "value": "Number (mm)",
          "method": "ultrasonic | coring"
        }
      }
    }
  }
}
```

### 7.2 Surface Quality

```json
{
  "surface": {
    "roughness": {
      "max": "Number (mm)",
      "measurement": "Ra | Rz"
    },
    "waviness": {
      "max": "Number (mm)",
      "wavelength": "Number (mm)"
    }
  }
}
```

### 7.3 Structural Testing

```json
{
  "structural": {
    "loadTesting": {
      "required": "Boolean",
      "loadFactor": "Number [1.0, 2.0]",
      "duration": "Number (hours)",
      "acceptance": "String (description)"
    }
  }
}
```

### 7.4 Inspection Frequency

| Inspection Type | Frequency | Method |
|----------------|-----------|---------|
| Visual | Per layer | Direct observation |
| Dimensional | Every 5 layers | Laser scan |
| Material | Per batch | Laboratory testing |
| Structural | Milestones | Load test, NDT |

## 8. Reinforcement Integration

Specification for both traditional and printed reinforcement.

### 8.1 Traditional Reinforcement

```json
{
  "reinforcement": {
    "traditional": [
      {
        "id": "String",
        "material": "steel-grade-60 | fiber-composite",
        "diameter": "Number (mm)",
        "path": "3D polyline coordinates",
        "placement": {
          "layer": "Integer",
          "pausePrint": "Boolean",
          "installMethod": "manual | robotic"
        }
      }
    ]
  }
}
```

### 8.2 Printed Reinforcement

```json
{
  "printed": [
    {
      "id": "String",
      "material": "fiber-composite | wire-mesh",
      "diameter": "Number (mm)",
      "distribution": "continuous-mesh | discrete-bars",
      "pattern": "rectilinear | diagonal | optimized",
      "density": "Number [0, 1]"
    }
  ]
}
```

## 9. Versioning and Compatibility

### 9.1 Semantic Versioning

Version format: `MAJOR.MINOR.PATCH`

- **MAJOR**: Breaking changes requiring file conversion
- **MINOR**: Backward-compatible new features
- **PATCH**: Bug fixes and clarifications

### 9.2 Version Compatibility

```json
{
  "version": "1.2.3",
  "minimumReaderVersion": "1.0.0",
  "deprecated": ["fieldName1", "fieldName2"],
  "extensions": {
    "vendor-namespace": { /* custom fields */ }
  }
}
```

### 9.3 Migration

For major version transitions, use official migration tools:
```bash
wia-convert --from 1.5.0 --to 2.0.0 project.json
```

## 10. Validation and Compliance

### 10.1 Syntactic Validation

Files MUST validate against JSON Schema:
```bash
wia-validate --schema project.schema.json project.json
```

### 10.2 Semantic Validation

Additional validation beyond syntax:
- Geometry manifoldness
- Material property plausibility
- Parameter compatibility
- Cross-reference integrity

### 10.3 Reference Implementation

Official Python validation library:
```python
from wia_standards import validate_project

result = validate_project('project.json')
if not result.valid:
    for error in result.errors:
        print(f"{error.path}: {error.message}")
```

## 11. Conformance Levels

### 11.1 Basic Conformance

REQUIRED:
- Valid JSON syntax
- Schema validation passes
- Required fields present
- Data types correct

### 11.2 Full Conformance

REQUIRED (in addition to Basic):
- Semantic validation passes
- Cross-references resolve
- Material properties within plausible ranges
- Geometric validity (manifold meshes)
- Parameter consistency

## 12. Security Considerations

### 12.1 Input Validation

Parsers MUST:
- Limit file size (default: 100 MB)
- Limit nesting depth (default: 20 levels)
- Validate numeric ranges
- Sanitize string inputs

### 12.2 Sensitive Data

Projects MAY contain:
- Proprietary designs
- Material formulations
- Client information

Implementations SHOULD support encryption at rest and in transit.

---

## Appendices

### Appendix A: Complete Schema Example

See `examples/complete-project.json` in reference implementation.

### Appendix B: Material Library

Standard material definitions available at:
`https://standards.wia.org/materials/`

### Appendix C: Validation Test Suite

Comprehensive test cases at:
`https://github.com/WIA-Official/wia-standards/tree/main/tests`

---

**Document Information:**
- **Version:** 1.0.0
- **Status:** Published
- **Date:** 2025-01-15
- **Maintained by:** WIA Technical Committee
- **License:** CC BY 4.0

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA
