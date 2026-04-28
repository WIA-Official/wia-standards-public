# WIA-AUTO-021 — Phase 1: Data Format

> Vehicle-lightweight-material canonical Phase 1: material + mechanical + manufacturing + joining + crash/recyclability envelopes.

# WIA-AUTO-021: Vehicle Lightweight Material Specification v1.0

> **Standard ID:** WIA-AUTO-021
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Materials Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Material Types](#2-material-types)
3. [Mechanical Properties](#3-mechanical-properties)
4. [Manufacturing Processes](#4-manufacturing-processes)
5. [Joining Technologies](#5-joining-technologies)
6. [Crash Safety Performance](#6-crash-safety-performance)
7. [Corrosion Resistance](#7-corrosion-resistance)
8. [Recyclability](#8-recyclability)
9. [Data Formats](#9-data-formats)
10. [API Interface](#10-api-interface)
11. [Testing Standards](#11-testing-standards)
12. [References](#12-references)

---


## 2. Material Types

### 2.1 Ferrous Materials

#### 2.1.1 High Strength Steel (HSS)

**Composition**: Low carbon steel with strengthening elements (Mn, Si, P)

**Properties**:
- Tensile strength: 270-550 MPa
- Yield strength: 180-420 MPa
- Elongation: 22-45%
- Density: 7,850 kg/m³

**Applications**: Body structure, chassis components, suspension parts

#### 2.1.2 Advanced High Strength Steel (AHSS)

**Types**:
- Dual Phase (DP) Steel: 500-1,200 MPa
- Complex Phase (CP) Steel: 700-1,000 MPa
- Transformation-Induced Plasticity (TRIP): 600-1,000 MPa
- Martensitic Steel: 900-1,700 MPa

**Properties**:
- Tensile strength: 500-1,700 MPa
- Excellent formability
- High energy absorption
- Density: 7,850 kg/m³

**Applications**: A/B-pillars, door beams, bumper reinforcements, roof rails

#### 2.1.3 Ultra High Strength Steel (UHSS)

**Composition**: Hot-stamped boron steel (22MnB5)

**Properties**:
- Tensile strength: 1,500-2,000 MPa
- Yield strength: 1,100-1,500 MPa
- Elongation: 4-8%
- Density: 7,850 kg/m³

**Applications**: Safety cage, door intrusion beams, B-pillars

### 2.2 Non-Ferrous Metals

#### 2.2.1 Aluminum Alloys

**Common Alloys**:

| Alloy | Type | Tensile Strength (MPa) | Yield Strength (MPa) | Elongation (%) | Applications |
|-------|------|----------------------|---------------------|----------------|--------------|
| 5052-H32 | Non-heat treatable | 228 | 193 | 12 | Body panels |
| 5754-H22 | Non-heat treatable | 220 | 150 | 18 | Inner panels |
| 6061-T6 | Heat treatable | 310 | 276 | 12 | Chassis, subframe |
| 6082-T6 | Heat treatable | 340 | 290 | 10 | Structural parts |
| 7075-T6 | Heat treatable | 572 | 503 | 11 | High-performance components |

**Properties**:
- Density: 2,700-2,810 kg/m³
- Young's Modulus: 69-72 GPa
- Thermal conductivity: 130-230 W/m·K
- Corrosion resistance: Excellent (with treatment)

**Weight Savings**: 40-50% vs. steel for equivalent strength

#### 2.2.2 Magnesium Alloys

**Common Alloys**:
- AZ31B (3% Al, 1% Zn): General purpose
- AZ91D (9% Al, 1% Zn): Die casting
- AM60B (6% Al, Mn): High ductility
- WE43 (4% Y, 3% rare earth): High temperature

**Properties**:
- Density: 1,770-1,840 kg/m³
- Tensile strength: 200-290 MPa
- Young's Modulus: 45 GPa
- Excellent damping characteristics

**Weight Savings**: 60-75% vs. steel, 35% vs. aluminum

**Limitations**: Lower corrosion resistance, requires protective coating

#### 2.2.3 Titanium Alloys

**Ti-6Al-4V (Grade 5)**:
- Tensile strength: 950 MPa
- Density: 4,430 kg/m³
- Young's Modulus: 114 GPa
- Excellent corrosion resistance
- High temperature capability

**Applications**: Exhaust systems, valve springs, fasteners (limited use due to cost)

### 2.3 Composite Materials

#### 2.3.1 Carbon Fiber Reinforced Plastic (CFRP)

**Fiber Types**:
- Standard Modulus (SM): 230 GPa
- Intermediate Modulus (IM): 290 GPa
- High Modulus (HM): 350-550 GPa

**Matrix Materials**:
- Epoxy resin: High strength, good adhesion
- Polyester resin: Lower cost, moderate properties
- Thermoplastic: Recyclable, fast processing

**Properties** (unidirectional, 60% fiber volume fraction):
- Tensile strength: 600-1,000 MPa (SM)
- Tensile modulus: 150 GPa
- Density: 1,550-1,600 kg/m³
- Specific strength: 375-625 kN·m/kg

**Weight Savings**: 50-60% vs. steel, 20-30% vs. aluminum

**Applications**: Monocoque chassis, body panels, driveshafts, suspension arms

#### 2.3.2 Glass Fiber Reinforced Plastic (GFRP)

**Fiber Types**:
- E-glass: General purpose, good electrical insulation
- S-glass: High strength and modulus
- C-glass: Chemical resistance

**Properties** (chopped strand mat):
- Tensile strength: 100-200 MPa
- Tensile modulus: 10-20 GPa
- Density: 1,800-2,000 kg/m³

**Properties** (woven fabric, 50% fiber volume):
- Tensile strength: 400-600 MPa
- Tensile modulus: 35-40 GPa
- Density: 1,900-2,100 kg/m³

**Applications**: Underbody panels, interior components, battery enclosures

#### 2.3.3 Natural Fiber Composites

**Fiber Types**: Flax, hemp, kenaf, jute
- Tensile strength: 500-900 MPa (individual fibers)
- Young's modulus: 50-70 GPa
- Density: 1,400-1,500 kg/m³
- Renewable and biodegradable

**Applications**: Interior trim, door panels, package trays

### 2.4 Advanced Materials

#### 2.4.1 Metal Matrix Composites (MMC)

**Aluminum MMC**:
- Matrix: Aluminum alloy
- Reinforcement: SiC, Al₂O₃ particles or fibers
- Properties: Enhanced stiffness and wear resistance
- Applications: Brake rotors, pistons, connecting rods

#### 2.4.2 Sandwich Structures

**Configuration**: Face sheets + core
- Face sheets: Aluminum, CFRP, steel
- Core: Honeycomb (aluminum, Nomex), foam (PU, PET)
- Benefits: High bending stiffness, low weight
- Applications: Floor panels, hoods, roofs

---



## 9. Data Formats

### 9.1 Material Property Data

#### 9.1.1 JSON Format

```json
{
  "material": {
    "id": "AL-6061-T6",
    "name": "Aluminum 6061-T6",
    "category": "Non-Ferrous Metal",
    "subcategory": "Aluminum Alloy",
    "composition": {
      "Al": 97.9,
      "Mg": 1.0,
      "Si": 0.6,
      "Cu": 0.28,
      "Cr": 0.2
    },
    "mechanical": {
      "density": {
        "value": 2700,
        "unit": "kg/m³"
      },
      "tensileStrength": {
        "value": 310,
        "unit": "MPa",
        "testStandard": "ASTM E8"
      },
      "yieldStrength": {
        "value": 276,
        "unit": "MPa",
        "offset": 0.2
      },
      "elongation": {
        "value": 12,
        "unit": "%",
        "gaugeLength": 50
      },
      "youngsModulus": {
        "value": 69,
        "unit": "GPa"
      },
      "poissonsRatio": 0.33,
      "hardness": {
        "value": 95,
        "scale": "Brinell"
      }
    },
    "thermal": {
      "thermalConductivity": {
        "value": 167,
        "unit": "W/m·K",
        "temperature": 25
      },
      "specificHeat": {
        "value": 896,
        "unit": "J/kg·K"
      },
      "thermalExpansion": {
        "value": 23.6,
        "unit": "10⁻⁶/K",
        "temperatureRange": [20, 100]
      },
      "meltingPoint": {
        "value": 652,
        "unit": "°C"
      }
    },
    "applications": [
      "Chassis components",
      "Suspension parts",
      "Wheels",
      "Structural extrusions"
    ],
    "advantages": [
      "Good strength-to-weight ratio",
      "Excellent corrosion resistance",
      "Weldable",
      "Machinable"
    ],
    "limitations": [
      "Lower strength than steel",
      "Moderate cost",
      "Requires protective coating in some environments"
    ]
  }
}
```

#### 9.1.2 Material Comparison

```json
{
  "comparison": {
    "materials": ["Steel-Mild", "AHSS-DP780", "AL-6061-T6", "CFRP-Standard"],
    "metrics": {
      "density": [7850, 7850, 2700, 1600],
      "tensileStrength": [400, 780, 310, 800],
      "specificStrength": [51, 99, 115, 500],
      "youngsModulus": [200, 200, 69, 150],
      "cost": [1.0, 1.8, 4.2, 25.0]
    },
    "units": {
      "density": "kg/m³",
      "tensileStrength": "MPa",
      "specificStrength": "kN·m/kg",
      "youngsModulus": "GPa",
      "cost": "relative (steel = 1.0)"
    }
  }
}
```

### 9.2 Weight Reduction Calculation

```json
{
  "weightReduction": {
    "component": "Hood",
    "originalMaterial": "Steel-Mild",
    "newMaterial": "AL-6061-T6",
    "geometry": {
      "surfaceArea": 1.8,
      "thickness": {
        "original": 0.8,
        "new": 1.2
      },
      "units": {
        "area": "m²",
        "thickness": "mm"
      }
    },
    "calculations": {
      "volumeOriginal": 0.00144,
      "volumeNew": 0.00216,
      "massOriginal": 11.3,
      "massNew": 5.83,
      "weightSaved": 5.47,
      "percentReduction": 48.4
    },
    "fuelSavings": {
      "vehicleWeight": 1500,
      "efficiencyFactor": 0.65,
      "fuelReduction": 0.24
    },
    "units": {
      "volume": "m³",
      "mass": "kg",
      "fuelReduction": "%"
    }
  }
}
```

---




---

## A.1 Material-record envelope

The Phase 1 envelope groups material records by family (high-strength steel — DP / TRIP / TWIP / press-hardened boron steel / martensitic; aluminium — 5xxx series for body panels, 6xxx for chassis, 7xxx for high-strength structural; magnesium — AZ31 / AZ91 / AM60 / AE44; titanium — Ti-6Al-4V Grade 5 for high-end suspension and exhaust; engineering plastics — PA6 / PA66 / PP-GF / PEEK; composites — CFRP / GFRP / hybrid; bio-composites — flax / hemp / kenaf reinforcements). Records carry the canonical fields: chemical composition (ASTM / EN / JIS / KS designation), density in g/cm^3, Young's modulus in GPa, yield and ultimate tensile strength in MPa, fatigue limit at 10^7 cycles, fracture toughness K_IC in MPa·m^0.5, and the strain-rate envelope (quasi-static, intermediate, dynamic per the SAE J2749 strain-rate-dependence envelope).

## A.2 Mechanical-property descriptor

A mechanical-property descriptor MUST list the test method (ASTM E8 / ISO 6892 tensile; ASTM E23 / ISO 148 Charpy impact; ASTM E647 / ISO 12108 fatigue-crack growth; ASTM E1820 / ISO 12135 fracture toughness; ASTM D3039 / ISO 527 composite tensile), the specimen geometry envelope, the conditioning envelope (temperature, humidity, pre-fatigue cycles), the test machine of record envelope, and the calibration record reference per ISO/IEC 17025. Anisotropic materials (rolled steel, extruded aluminium, oriented composites) carry the per-direction property matrix (RD / TD / ND for sheet; per-laminate-orientation for composites).

## A.3 Manufacturing-process descriptor

The manufacturing-process descriptor enumerates: stamping / hot-stamping / hot-forming with quench (PHS / press-hardened steel) / superplastic forming / hydroforming / roll forming / injection moulding / RTM / pultrusion / autoclave-cure / out-of-autoclave / 3D printing (SLM / DED / FDM / SLA). Each entry carries the cycle-time envelope, the energy-consumption envelope per part, the scrap-rate envelope, the dimensional-tolerance envelope per ISO 2768 / GD&T per ASME Y14.5, and the surface-quality envelope.

## A.4 Joining-technology descriptor

A joining-technology descriptor enumerates: spot welding (RSW), laser welding, friction-stir welding (FSW), self-piercing rivets (SPR), flow-drill screws (FDS), structural adhesive bonding (epoxy / urethane / acrylic), clinching, and hybrid joining (e.g., adhesive + SPR). Each entry carries the joint-strength envelope (lap-shear, peel, fatigue), the post-paint corrosion-class envelope, the dissimilar-material compatibility envelope (steel-aluminium galvanic-corrosion mitigation per SAE J2747), and the inspection-method envelope (visual / ultrasonic / radiographic / shear-tear / dye-penetrant).

## A.5 Crashworthiness and recyclability envelope

Crashworthiness envelopes carry: per-component energy-absorption per Euro NCAP / IIHS / KNCAP / Euro NCAP small-overlap / FMVSS 208 / FMVSS 214 envelopes; the deformation-mode envelope (axial crush, bending, splitting); the strain-rate-dependent material model identifier; and the post-crash repairability classification. Recyclability envelopes follow ISO 22628 (vehicle recyclability and recoverability calculation) and the EU End-of-Life Vehicles Directive 2000/53/EC: per-component recyclate content, the end-of-life dismantling envelope, and the material-passport reference (Battery Passport per EU Regulation 2023/1542 for EV battery enclosures, vehicle material passport per the proposed EU Digital Product Passport).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/vehicle-lightweight-material/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-vehicle-lightweight-material-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/vehicle-lightweight-material-host:1.0.0` ships every vehicle-lightweight-material envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/vehicle-lightweight-material.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Vehicle-lightweight-material deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
