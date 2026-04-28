# WIA-AUTO-021 — Phase 3: Protocol

> Vehicle-lightweight-material canonical Phase 3: protocols (manufacturing + joining + crash-test + corrosion + recyclability).

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


## 4. Manufacturing Processes

### 4.1 Metal Forming

#### 4.1.1 Stamping and Deep Drawing

**Process Parameters**:
- Blank holder force: 5-30% of punch force
- Drawing ratio (DR): 1.8-2.2 (steel), 1.6-1.8 (aluminum)
- Punch radius: r_p ≥ 6t (steel), r_p ≥ 8t (aluminum)
- Die radius: r_d ≥ 4t

**Applicable Materials**:
- Mild steel, HSS
- Aluminum 5xxx, 6xxx series (annealed)
- Drawing-quality steel (DQ, DDQ, EDDQ)

#### 4.1.2 Hot Stamping (Press Hardening)

**Process** (22MnB5 boron steel):
1. Heating: 900-950°C, 3-10 minutes (austenitizing)
2. Transfer: < 5 seconds
3. Forming + Quenching: In-die, 150-250°C, 5-20 seconds
4. Cooling rate: > 27°C/s (martensite formation)

**Results**:
- Tensile strength: 1,500 MPa → 1,900 MPa
- Minimal springback
- Excellent dimensional accuracy

**Applications**: A/B-pillars, roof rails, door beams, tunnel reinforcement

#### 4.1.3 Superplastic Forming (SPF)

**Materials**: Aluminum 5083 (SPF grade), Titanium Ti-6Al-4V

**Process Parameters**:
- Temperature: 450-550°C (Al), 850-950°C (Ti)
- Strain rate: 10⁻⁴ to 10⁻² s⁻¹
- Gas pressure: 1-2 MPa
- Elongation capability: 200-800%

**Benefits**: Complex shapes, low tooling cost, no springback

#### 4.1.4 Incremental Sheet Forming (ISF)

**Process**: CNC-controlled forming with simple punch tool
- No dedicated dies required
- Suitable for prototyping and low-volume production
- Formability: Higher than stamping (2-3× elongation)

### 4.2 Composite Manufacturing

#### 4.2.1 Hand Layup / Wet Layup

**Process**:
1. Mold preparation and release agent application
2. Layer-by-layer fiber placement
3. Resin application (brush, roller)
4. Consolidation (roller, squeegee)
5. Curing (room temperature or elevated)

**Fiber Volume Fraction**: 30-50%
**Applications**: Prototypes, low-volume production, large parts

#### 4.2.2 Resin Transfer Molding (RTM)

**Process**:
1. Dry fiber preform placement in mold
2. Mold closing
3. Resin injection (0.1-0.5 MPa)
4. Curing (80-150°C, 10-60 minutes)
5. Demolding

**Fiber Volume Fraction**: 50-60%
**Cycle Time**: 5-20 minutes
**Applications**: Structural components, body panels (medium volume)

#### 4.2.3 Autoclave Molding

**Process**:
1. Prepreg layup on mold
2. Vacuum bagging
3. Autoclave curing:
   - Pressure: 0.6-0.7 MPa
   - Temperature: 120-180°C (epoxy)
   - Time: 2-8 hours

**Fiber Volume Fraction**: 55-65%
**Properties**: Excellent (aerospace quality)
**Limitations**: High cost, long cycle time

#### 4.2.4 Compression Molding (SMC/BMC)

**Materials**:
- SMC (Sheet Molding Compound): Chopped fibers in thermoset resin
- BMC (Bulk Molding Compound): Similar, paste form

**Process**:
1. Material placement in heated mold
2. Compression (5-15 MPa)
3. Curing (140-160°C, 1-5 minutes)
4. Demolding

**Fiber Volume Fraction**: 20-40%
**Cycle Time**: 2-10 minutes
**Applications**: High-volume production (hoods, deck lids, fenders)

#### 4.2.5 High-Pressure RTM (HP-RTM)

**Process Parameters**:
- Injection pressure: 10-100 MPa
- Cycle time: 1-3 minutes
- Fiber volume fraction: 50-60%

**Benefits**: Automotive-compatible cycle times, excellent properties
**Applications**: Structural components for electric vehicles

### 4.3 Casting

#### 4.3.1 High-Pressure Die Casting (HPDC)

**Materials**: Aluminum (A380, A383), Magnesium (AM60, AZ91)

**Process**:
- Injection pressure: 50-150 MPa
- Injection speed: 30-100 m/s
- Cycle time: 30-90 seconds

**Applications**: Complex thin-walled parts (housings, brackets, engine blocks)

#### 4.3.2 Vacuum Die Casting (VDC)

**Improvement over HPDC**:
- Vacuum: < 100 mbar
- Reduced porosity
- Heat treatable (T6, T7)
- Higher mechanical properties

### 4.4 Additive Manufacturing

#### 4.4.1 Selective Laser Melting (SLM)

**Materials**: AlSi10Mg, Ti-6Al-4V, Maraging Steel

**Process**:
- Layer thickness: 20-100 μm
- Laser power: 100-400 W
- Scanning speed: 500-2,000 mm/s

**Applications**: Complex lightweight structures, topology-optimized parts, low-volume production

---



## 5. Joining Technologies

### 5.1 Mechanical Joining

#### 5.1.1 Self-Piercing Rivet (SPR)

**Process**:
1. Rivet pierces top sheet
2. Spreads in bottom sheet without breakthrough
3. Mechanical interlock

**Parameters**:
- Rivet length: Based on total stack thickness
- Setting force: 20-40 kN
- Die depth: 1.0-2.0 mm

**Joint Strength** (2 mm Al + 2 mm Al):
- Tensile-shear: 3-5 kN
- Cross-tension: 1.5-3 kN

**Applications**: Aluminum-to-aluminum, aluminum-to-steel, multi-material joints

#### 5.1.2 Flow Drill Screw (FDS)

**Process**:
1. Friction drilling creates bushing
2. Thread forming
3. Screw fastening

**Benefits**: No pre-drilling, increased thread engagement, disassemble-able

#### 5.1.3 Clinching

**Process**: Mechanical joining by local plastic deformation (no consumables)

**Types**:
- Round clinching
- Rectangular clinching
- Hybrid clinching (with adhesive)

**Joint Strength**: 1-4 kN (depending on materials and geometry)

### 5.2 Welding

#### 5.2.1 Resistance Spot Welding (RSW)

**Process**:
- Current: 5-20 kA
- Time: 0.1-0.5 seconds
- Electrode force: 2-6 kN

**Applications**: Steel-to-steel (primary method for body-in-white)

**Limitations**: Difficult for aluminum (high conductivity), limited for multi-material

#### 5.2.2 Laser Welding

**Types**:
- Conduction mode: Lower power, cosmetic welds
- Keyhole mode: Higher power, deep penetration

**Parameters**:
- Power: 1-6 kW
- Speed: 1-10 m/min
- Focus diameter: 0.2-0.6 mm

**Benefits**: Deep penetration, narrow HAZ, high speed, minimal distortion

**Applications**: Tailored blanks, roof seams, door frames

#### 5.2.3 Friction Stir Welding (FSW)

**Process**:
- Rotating tool plunges into joint line
- Frictional heat plasticizes material
- Material stirring creates solid-state weld

**Parameters**:
- Rotational speed: 400-1,500 rpm
- Travel speed: 50-500 mm/min
- Plunge force: 10-40 kN

**Benefits**: No melting, no fumes, excellent properties, dissimilar materials

**Applications**: Aluminum alloy joining (space frames, battery trays, closures)

#### 5.2.4 Remote Laser Welding (RLW)

**Process**:
- Galvanometer scanner directs beam
- No physical contact with part
- Rapid repositioning

**Benefits**: Accessibility, flexibility, reduced cycle time

### 5.3 Adhesive Bonding

#### 5.3.1 Structural Adhesives

**Types**:
- Epoxy: High strength, long cure time
- Polyurethane: Flexible, moderate strength
- Acrylic: Fast cure, good impact resistance
- Crash-resistant adhesives: Combine stiffness with energy absorption

**Properties** (typical epoxy):
- Shear strength: 15-30 MPa
- Peel strength: 5-15 N/mm
- Cure temperature: 180-200°C (in E-coat oven)

**Benefits**: Uniform stress distribution, vibration damping, corrosion prevention

**Applications**: Hemming (doors, hoods), panel bonding, stiffening patches

#### 5.3.2 Adhesive + Mechanical (Rivet-bonding, Weld-bonding)

**Concept**: Adhesive provides static strength, fastener provides handling strength and crash performance

**Benefits**:
- Immediate handling strength
- Enhanced fatigue life
- Redundancy (safety)

### 5.4 Multi-Material Joining Strategies

**Challenges**: Different melting points, thermal expansion, galvanic corrosion

**Solutions**:

| Joint Combination | Preferred Method | Alternatives |
|-------------------|------------------|--------------|
| Steel - Steel | RSW, Laser | Adhesive, SPR |
| Al - Al | FSW, SPR | Laser, RSW (special electrodes) |
| Al - Steel | SPR, FDS | Laser brazing, FSW |
| CFRP - CFRP | Adhesive | Mechanical (bolts, rivets) |
| CFRP - Metal | Adhesive + Rivet | FDS, SPR (with protection) |
| Mg - Steel | Adhesive + Mechanical | — |

---



## 11. Testing Standards

### 11.1 Mechanical Testing

| Test | Standard | Purpose |
|------|----------|---------|
| Tensile Test | ISO 6892-1, ASTM E8 | Strength, ductility, modulus |
| Compression Test | ASTM E9 | Compressive properties |
| Shear Test | ASTM B769 | Shear strength |
| Hardness Test | ISO 6506 (Brinell), ISO 6507 (Vickers), ASTM E18 (Rockwell) | Material hardness |
| Impact Test | ISO 148-1, ASTM E23 | Toughness, impact resistance |
| Fatigue Test | ISO 1099, ASTM E466 | Fatigue life, S-N curve |
| Creep Test | ASTM E139 | Time-dependent deformation |
| Fracture Toughness | ASTM E399 | K_IC, critical stress intensity |

### 11.2 Formability Testing

| Test | Standard | Purpose |
|------|----------|---------|
| Erichsen Test | ISO 20482 | Sheet formability, cupping depth |
| Limiting Dome Height (LDH) | ISO 12004-2 | Formability limit |
| Hole Expansion Test | ISO 16630 | Edge stretchability |
| Bend Test | ASTM E290, VDA 238-100 | Bendability, minimum radius |

### 11.3 Corrosion Testing

| Test | Standard | Purpose |
|------|----------|---------|
| Salt Spray Test | ASTM B117, ISO 9227 | Continuous salt exposure |
| Cyclic Corrosion Test | SAE J2334, VDA 621-415 | Realistic exposure cycles |
| Humidity Test | ASTM D2247 | Condensation resistance |
| Scribe Creep Test | ASTM D1654 | Paint adhesion, under-film corrosion |

### 11.4 Composite Testing

| Test | Standard | Purpose |
|------|----------|---------|
| Fiber Volume Fraction | ASTM D3171 | Resin burnoff / acid digestion |
| Interlaminar Shear | ASTM D2344 | Short beam strength |
| Compression After Impact (CAI) | ASTM D7137 | Damage tolerance |
| Open Hole Tension/Compression | ASTM D5766, D6484 | Notch sensitivity |

### 11.5 Crash and Safety Testing

| Test | Standard | Purpose |
|------|----------|---------|
| Frontal Impact | FMVSS 208, Euro NCAP | Full-scale frontal crash |
| Side Impact | FMVSS 214, Euro NCAP | Pole and barrier side impact |
| Roof Crush | FMVSS 216 | Static roof strength |
| Component Crush | Internal OEM standards | Energy absorption, SEA |

---




---

## A.1 Manufacturing-process protocol

Stamping protocol: blank holder force, die radius, draw depth, friction coefficient envelope per the operator's lubrication tribology, springback compensation factor per the material's hardening curve, and the post-stamping inspection (3D laser scan vs. CAD with the documented tolerance band per ISO 2768 / ASME Y14.5). Hot-stamping (PHS / press-hardened steel) protocol adds the austenitisation cycle (typically 900-950 C for 4-6 minutes for 22MnB5), the in-die quench envelope (cooling rate >27 K/s), the post-quench tempering envelope, and the mechanical-property verification window (UTS 1500-1800 MPa typical for 22MnB5). Composite RTM protocol covers preform layup, mould injection envelope, cure schedule, and the post-cure NDT envelope per the operator's defect-acceptance criteria.

## A.2 Joining-technology protocol

Spot-welding (RSW) protocol: electrode force, weld current, weld time, hold time, with the per-stack-up envelope honouring AWS D8.7 / D8.9 industrial automotive welding standards. Laser-welding protocol covers spot- vs. seam-mode, beam-shape envelope, shielding-gas envelope, and the seam-monitoring envelope (penetration sensing via plasma/keyhole optics). FSW protocol covers tool-geometry envelope (probe length, shoulder diameter), tool-rotation and traverse rates, downward forge force, and the joint-line tracking accuracy. Adhesive-bonding protocol covers surface-preparation per ASTM D2651 (steel) / ASTM D7234 (aluminium / composite), bondline thickness and gap, cure schedule, and the per-batch lap-shear acceptance test per ASTM D1002 / ISO 4587.

## A.3 Crash-test and certification protocol

Crash-test protocol covers the regulatory crash matrix (FMVSS 208 frontal full-overlap; FMVSS 214 side-impact moving deformable barrier; FMVSS 301 rear-impact fuel system integrity; FMVSS 226 ejection mitigation) plus the consumer-rating crash matrix (Euro NCAP / IIHS / KNCAP / JNCAP / Latin NCAP / ASEAN NCAP). Each test is preceded by the FE pre-test prediction with the validation-residual envelope; post-test the realised acceleration and deformation traces are compared against the prediction with the documented acceptance band. Repeat tests are required when prediction-residuals exceed the band per the operator's design-validation protocol.

## A.4 Corrosion-test protocol

Corrosion-test protocol covers cyclic accelerated corrosion (SAE J2334; GMW14872 GMW; VDA 233-102 cyclic; KSR 4321 cyclic equivalents) with the documented exposure cycle, the post-exposure inspection envelope (red-rust grading per ASTM D610 / ISO 4628; perforation-rate envelope), and the dissimilar-material galvanic-corrosion envelope (per SAE J2747 with the specified gap/insulation/sealant profile). Outdoor proving-ground correlation factors are documented per the operator's atmospheric-correlation envelope so accelerated-test results can be extrapolated to in-service exposure expectations.

## A.5 Recyclability protocol

The recyclability protocol enforces ISO 22628 calculation conventions: per-component material breakdown, the recoverability-and-recyclability envelope per the EU ELV Directive 2000/53/EC's 95% recovery / 85% recycling targets, and the end-of-life dismantling envelope (ATF / authorised treatment facility instructions, hazardous-component pre-treatment per the EU ELV Annex II prohibited substances). For EV propulsion batteries the recyclability envelope additionally references the EU Battery Regulation 2023/1542 (recyclate content thresholds for cobalt / lithium / nickel / lead from the documented effective dates).

## A.6 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for the test-result registration control plane. Test-result records are signed at registration time and the signature chain is anchored into a Merkle tree per-operator so revisions to the test history can be detected during post-build investigations. Test-machine telemetry uses mTLS with per-machine monotonic counters; replay attempts are detected and dropped at the broker.


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
