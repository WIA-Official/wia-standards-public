# WIA-IND-029 PHASE 2 — API Interface Specification

**Standard:** WIA-IND-029
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

<model unit="millimeter" xml:lang="en-US">
  <resources>
    <object id="1" type="model">
      <mesh>
        <vertices>
          <vertex x="0" y="0" z="0"/>
          <vertex x="100" y="0" z="0"/>
          <vertex x="0" y="100" z="0"/>
        </vertices>
        <triangles>
          <triangle v1="0" v2="1" v3="2"/>
        </triangles>
      </mesh>
    </object>
  </resources>
  <build>
    <item objectid="1"/>
  </build>
</model>
```

#### 3.1.3 OBJ (Wavefront Object)

**Features:**
- Widely supported
- Color and texture mapping
- ASCII format (human-readable)

#### 3.1.4 AMF (Additive Manufacturing File)

**Features:**
- XML-based
- Curved triangles (higher accuracy)
- Color, material, and texture support
- Metadata storage

#### 3.1.5 STEP (Standard for Exchange of Product Data)

**Features:**
- CAD native format
- Parametric geometry (not mesh)
- Precise for engineering applications
- Requires conversion to mesh for printing

### 3.2 Mesh Processing

#### 3.2.1 Mesh Validation

**Common Issues:**
- Non-manifold edges (edges shared by >2 faces)
- Holes and gaps in mesh
- Inverted normals
- Intersecting triangles
- Degenerate triangles (zero area)

**Validation Process:**
```json
{
  "validation": {
    "manifold": true,
    "watertight": true,
    "normalConsistency": true,
    "minimumTriangleArea": 0.001,
    "maximumAspectRatio": 100
  },
  "repair": {
    "autoFix": true,
    "fillHoles": true,
    "removeIntersections": true,
    "flipNormals": "auto"
  }
}
```

#### 3.2.2 Mesh Optimization

**Techniques:**
- **Decimation**: Reduce triangle count while preserving shape
- **Smoothing**: Remove surface artifacts
- **Remeshing**: Create uniform triangle distribution
- **Simplification**: Reduce complexity for faster slicing

```json
{
  "optimization": {
    "targetTriangleCount": 50000,
    "edgeLength": { "min": 0.1, "max": 2.0 },
    "preserveFeatures": true,
    "preserveBoundaries": true,
    "smoothingIterations": 3
  }
}
```

### 3.3 Model Transformations

#### 3.3.1 Scaling, Rotation, Translation

```json
{
  "transform": {
    "scale": { "x": 1.0, "y": 1.0, "z": 1.0 },
    "rotate": { "x": 0, "y": 0, "z": 45 },
    "translate": { "x": 50, "y": 50, "z": 0 },
    "units": {
      "source": "inches",
      "target": "millimeters",
      "autoConvert": true
    }
  }
}
```

#### 3.3.2 Model Placement Optimization

**Auto-Orientation:**
- Minimize support material
- Maximize strength (layer orientation)
- Optimize surface finish
- Reduce print time

```json
{
  "placement": {
    "algorithm": "ai-optimize",
    "objectives": {
      "minimizeSupport": 0.4,
      "maximizeStrength": 0.3,
      "minimizePrintTime": 0.3
    },
    "constraints": {
      "overhangAngle": 45,
      "maxSupportVolume": 50,
      "criticalSurfaces": ["top", "front"]
    }
  }
}
```

---

## 4. Slicing Algorithms and Optimization

### 4.1 Slicing Process

#### 4.1.1 Layer Generation

**Algorithm:**
1. Intersect model mesh with horizontal planes (Z-heights)
2. Generate 2D contours for each layer
3. Determine perimeters (walls)
4. Calculate infill regions
5. Generate support structures
6. Create G-code toolpaths

#### 4.1.2 Perimeter Generation

```json
{
  "perimeters": {
    "count": 3,
    "orderStrategy": "outside-in",
    "overlapPercentage": 25,
    "minPerimeterWidth": 0.35,
    "externalFirst": false,
    "gapFillSpeed": 20,
    "thinWallsEnabled": true
  }
}
```

### 4.2 Infill Patterns

#### 4.2.1 Pattern Types

| Pattern | Strength | Speed | Material Usage | Best For |
|---------|----------|-------|----------------|----------|
| Grid | Medium | Fast | Low | General purpose |
| Lines | Low | Fastest | Lowest | Non-structural |
| Triangles | High | Slow | Medium | Structural parts |
| Tri-Hexagon | High | Medium | Medium | Balanced |
| Gyroid | Very High | Medium | Medium | Organic strength |
| Honeycomb | High | Slow | Low | Lightweight |
| Cubic | High | Medium | Medium | Isotropic |
| Octet | Very High | Slow | Medium | Maximum strength |
| Concentric | Medium | Medium | Low | Flexible parts |
| Hilbert Curve | Medium | Slow | Medium | Aesthetic |

#### 4.2.2 Adaptive Infill

Variable density infill based on stress analysis:

```json
{
  "infill": {
    "type": "adaptive",
    "densityRange": { "min": 5, "max": 80 },
    "algorithm": "stress-based",
    "stressAnalysis": {
      "loadCases": [
        { "force": [0, 0, -1000], "point": [50, 50, 100] }
      ],
      "safetyFactor": 2.0,
      "materialProperties": {
        "yieldStrength": 50,
        "elasticModulus": 3500
      }
    }
  }
}
```

### 4.3 Support Generation

#### 4.3.1 Support Types

**Standard Supports:**
- Grid pattern
- Lines
- Zig-zag

**Tree Supports:**
- Organic branching structure
- Minimal contact with model
- Easier removal
- Less scarring

**AI-Generated Supports:**
- Machine learning optimization
- Minimal material usage
- Optimal contact points
- Easy removal paths

```json
{
  "supports": {
    "enabled": true,
    "type": "tree-ai",
    "overhangAngle": 45,
    "pattern": "gyroid",
    "density": 15,
    "contactZDistance": 0.2,
    "contactXYDistance": 0.7,
    "branchAngle": 60,
    "branchDiameter": 2.0,
    "minimumSupport": 5.0,
    "connectSupports": true
  }
}
```

### 4.4 Advanced Slicing Features

#### 4.4.1 Variable Layer Height

Adjust layer height based on model geometry:
- Thin layers for curved surfaces (detail)
- Thick layers for flat regions (speed)

```json
{
  "variableLayerHeight": {
    "enabled": true,
    "minLayerHeight": 0.08,
    "maxLayerHeight": 0.3,
    "adaptiveAlgorithm": "curvature-based",
    "transitionLayers": 3,
    "smoothTransitions": true
  }
}
```

#### 4.4.2 Ironing

Top surface finishing for glass-like finish:

```json
{
  "ironing": {
    "enabled": true,
    "flowRate": 10,
    "speed": 20,
    "spacing": 0.1,
    "pattern": "zig-zag"
  }
}
```

#### 4.4.3 Seam Control

Control where layer changes occur:

```json
{
  "seam": {
    "position": "aligned",
    "cornerPreference": "smart-hiding",
    "location": { "x": "rear", "y": "left" }
  }
}
```

---

## 5. Material Specifications

### 5.1 Material Properties Database

#### 5.1.1 Thermoplastic Properties

**PLA (Polylactic Acid):**
```json
{
  "material": "PLA",
  "category": "thermoplastic",
  "properties": {
    "printTemperature": { "min": 190, "max": 220, "optimal": 205 },
    "bedTemperature": { "min": 0, "max": 60, "optimal": 50 },
    "density": 1.24,
    "tensileStrength": 50,
    "elongationAtBreak": 6,
    "flexuralModulus": 3500,
    "heatDeflectionTemp": 60,
    "thermalExpansion": 68,
    "printSpeed": { "min": 40, "max": 80, "optimal": 60 }
  },
  "characteristics": {
    "biodegradable": true,
    "foodSafe": true,
    "warpResistance": "high",
    "bridging": "excellent",
    "overhangAngle": 50,
    "moistureSensitive": false
  },
  "applications": [
    "prototypes",
    "models",
    "educational",
    "decorative"
  ]
}
```

**PETG (Polyethylene Terephthalate Glycol):**
```json
{
  "material": "PETG",
  "properties": {
    "printTemperature": { "optimal": 235 },
    "bedTemperature": { "optimal": 80 },
    "tensileStrength": 53,
    "elongationAtBreak": 140,
    "heatDeflectionTemp": 80,
    "impactStrength": "high",
    "chemicalResistance": "good"
  },
  "characteristics": {
    "foodSafe": true,
    "recyclable": true,
    "warpResistance": "medium",
    "layerAdhesion": "excellent",
    "moistureSensitive": true,
    "stringing": "medium"
  }
}
```

#### 5.1.2 Engineering Polymers

**Nylon (PA12):**
```json
{
  "material": "PA12",
  "properties": {
    "printTemperature": { "optimal": 260 },
    "bedTemperature": { "optimal": 85 },
    "tensileStrength": 75,
    "elongationAtBreak": 50,
    "flexuralModulus": 1600,
    "abrasionResistance": "excellent"
  },
  "characteristics": {
    "hygroscopic": true,
    "dryingRequired": { "temp": 80, "time": 12, "humidity": "<10%" },
    "chemicalResistance": "excellent",
    "warpResistance": "poor",
    "enclosureRequired": true
  }
}


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.
