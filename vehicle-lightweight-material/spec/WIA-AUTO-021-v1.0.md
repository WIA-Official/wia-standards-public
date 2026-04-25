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

## 1. Introduction

### 1.1 Purpose

This specification defines the standards for lightweight materials used in automotive applications, including material properties, testing methods, manufacturing processes, and performance requirements.

### 1.2 Scope

The standard covers:
- Material specifications for lightweight automotive materials
- Mechanical property requirements and testing
- Manufacturing process guidelines
- Joining and assembly methods
- Safety and crash performance standards
- Environmental and recyclability considerations
- Data formats and API interfaces for material databases

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard promotes the development and adoption of lightweight materials that reduce vehicle emissions, improve fuel efficiency, and enhance safety while ensuring environmental sustainability and economic viability.

### 1.4 Terminology

- **Specific Strength**: Strength-to-weight ratio (σ/ρ)
- **Specific Modulus**: Stiffness-to-weight ratio (E/ρ)
- **CFRP**: Carbon Fiber Reinforced Plastic/Polymer
- **GFRP**: Glass Fiber Reinforced Plastic/Polymer
- **HSS**: High Strength Steel
- **AHSS**: Advanced High Strength Steel
- **UHSS**: Ultra High Strength Steel
- **RTM**: Resin Transfer Molding
- **SPR**: Self-Piercing Rivet
- **FSW**: Friction Stir Welding

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

## 3. Mechanical Properties

### 3.1 Strength Properties

#### 3.1.1 Tensile Strength

Test method: ISO 6892-1, ASTM E8

**Formula**:
```
σ_tensile = F_max / A₀
```

Where:
- `σ_tensile` = Tensile strength (MPa)
- `F_max` = Maximum force (N)
- `A₀` = Original cross-sectional area (mm²)

**Minimum Requirements** (by application):
- Body panels: 140 MPa
- Structural components: 270 MPa
- Safety-critical parts: 600 MPa

#### 3.1.2 Yield Strength

**Formula**:
```
σ_yield = F_0.2 / A₀
```

Where:
- `σ_yield` = 0.2% offset yield strength (MPa)
- `F_0.2` = Force at 0.2% plastic strain (N)

#### 3.1.3 Specific Strength

**Formula**:
```
σ_specific = σ_tensile / ρ
```

Where:
- `σ_specific` = Specific strength (kN·m/kg)
- `ρ` = Density (kg/m³)

**Performance Index**:
- Steel (mild): 51 kN·m/kg
- AHSS: 191 kN·m/kg
- Aluminum 7075-T6: 204 kN·m/kg
- CFRP: 375-625 kN·m/kg (best performance)

### 3.2 Stiffness Properties

#### 3.2.1 Young's Modulus (Elastic Modulus)

Test method: ISO 6892-1

**Formula**:
```
E = σ / ε = (ΔF / A₀) / (ΔL / L₀)
```

Where:
- `E` = Young's modulus (GPa)
- `σ` = Stress (MPa)
- `ε` = Strain (dimensionless)
- `ΔF` = Force increment (N)
- `ΔL` = Length change (mm)
- `L₀` = Original length (mm)

**Typical Values**:
- Steel: 200-210 GPa
- Aluminum: 69-72 GPa
- Magnesium: 45 GPa
- CFRP: 150-180 GPa
- Titanium: 114 GPa

#### 3.2.2 Specific Modulus

**Formula**:
```
E_specific = E / ρ
```

Where:
- `E_specific` = Specific modulus (MN·m/kg)

**Performance Index**:
- Steel: 25.5 MN·m/kg
- Aluminum: 25.6 MN·m/kg
- Magnesium: 25.4 MN·m/kg
- CFRP: 94-112 MN·m/kg (superior stiffness-to-weight)

### 3.3 Ductility and Formability

#### 3.3.1 Elongation at Break

Test method: ISO 6892-1

**Formula**:
```
A = (L_f - L₀) / L₀ × 100%
```

Where:
- `A` = Elongation (%)
- `L_f` = Final gauge length (mm)
- `L₀` = Original gauge length (mm)

**Minimum Requirements**:
- Cold forming: A ≥ 20%
- Deep drawing: A ≥ 30%
- Safety-critical (UHSS): A ≥ 4%

#### 3.3.2 Forming Limit Diagram (FLD)

Test method: ISO 12004-2

- Maps the formability limits in biaxial strain space
- Major strain vs. minor strain
- Critical for stamping and deep drawing operations

### 3.4 Fatigue Properties

#### 3.4.1 S-N Curve (Wöhler Curve)

Test method: ISO 1099, ASTM E466

**Formula** (Basquin's equation):
```
σ_a = σ_f' × (2N_f)^b
```

Where:
- `σ_a` = Stress amplitude (MPa)
- `σ_f'` = Fatigue strength coefficient
- `N_f` = Number of cycles to failure
- `b` = Fatigue strength exponent

**Endurance Limit**:
- Steel: ~0.5 × σ_tensile (at 10⁷ cycles)
- Aluminum: No true endurance limit (continuous decrease)
- CFRP: Excellent fatigue resistance (90% of static strength at 10⁶ cycles)

#### 3.4.2 Crack Growth Rate

Test method: ASTM E647

**Paris Law**:
```
da/dN = C × (ΔK)^m
```

Where:
- `da/dN` = Crack growth rate (mm/cycle)
- `ΔK` = Stress intensity factor range (MPa·√m)
- `C`, `m` = Material constants

### 3.5 Impact and Energy Absorption

#### 3.5.1 Charpy Impact Test

Test method: ISO 148-1, ASTM E23

**Absorbed Energy**:
```
E = m × g × (h₀ - h₁)
```

Where:
- `E` = Absorbed energy (J)
- `m` = Pendulum mass (kg)
- `g` = Gravitational acceleration (9.81 m/s²)
- `h₀` = Initial height (m)
- `h₁` = Final height (m)

**Minimum Requirements**:
- Room temperature: E ≥ 27 J
- Low temperature (-40°C): E ≥ 18 J

#### 3.5.2 Specific Energy Absorption (SEA)

**Formula**:
```
SEA = E_absorbed / m_structure
```

Where:
- `SEA` = Specific energy absorption (kJ/kg)
- `E_absorbed` = Total absorbed energy (kJ)
- `m_structure` = Mass of structure (kg)

**Minimum Requirement**: SEA ≥ 50 kJ/kg (crash-critical components)

**Typical Values**:
- Aluminum honeycomb: 50-120 kJ/kg
- Aluminum foam: 5-15 kJ/kg
- CFRP tubes: 70-100 kJ/kg
- Steel tubes: 20-40 kJ/kg

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

## 6. Crash Safety Performance

### 6.1 Energy Absorption Mechanisms

#### 6.1.1 Axial Crushing

**Progressive Buckling** (thin-walled tubes):

**Mean Crushing Force**:
```
P_m = K × σ_0 × t^(5/3) × D^(1/3)
```

Where:
- `P_m` = Mean crushing force (N)
- `K` = Material and geometry constant (30-40)
- `σ_0` = Flow stress (MPa)
- `t` = Wall thickness (mm)
- `D` = Tube diameter (mm)

**Specific Energy Absorption**:
```
SEA = P_m × δ / (ρ × V)
```

Where:
- `δ` = Crushing distance (mm)
- `V` = Volume of crushed zone (mm³)

**Typical SEA Values**:
- Steel tubes: 20-40 kJ/kg
- Aluminum tubes: 25-50 kJ/kg
- CFRP tubes: 60-100 kJ/kg
- Aluminum foam-filled: 50-80 kJ/kg

#### 6.1.2 Bending Collapse

**Plastic Hinge Formation**:
```
M_p = σ_0 × Z
```

Where:
- `M_p` = Plastic moment (N·mm)
- `Z` = Plastic section modulus (mm³)

**Applications**: Door beams, bumper beams, B-pillars

### 6.2 Crash Test Standards

#### 6.2.1 Frontal Impact

**NCAP Test** (FMVSS 208, Euro NCAP):
- Impact speed: 56 km/h (35 mph)
- Barrier: Offset deformable (40% overlap)
- Metrics: Intrusion, HIC (Head Injury Criterion), chest acceleration

**Material Requirements**:
- High-strength A/B-pillars (UHSS, 1,500 MPa)
- Progressive crush zones (AHSS, DP, TRIP)
- Energy-absorbing components (aluminum, composites)

#### 6.2.2 Side Impact

**NCAP Test** (FMVSS 214, Euro NCAP):
- Impact speed: 50-60 km/h
- Barrier: Moving deformable barrier (MDB)
- Metrics: Chest deflection, pelvis acceleration

**Material Requirements**:
- Door beams (UHSS, CFRP): Tensile strength ≥ 1,200 MPa
- B-pillars (UHSS, hot-stamped): Tensile strength ≥ 1,500 MPa
- Energy-absorbing padding (foam, honeycomb)

#### 6.2.3 Roof Crush

**FMVSS 216**:
- Load requirement: ≥ 3.0 × vehicle weight
- Displacement: ≤ 127 mm (5 inches)

**Material Strategy**: High-strength roof rails and pillars (UHSS)

### 6.3 Pedestrian Protection

**Euro NCAP Pedestrian Testing**:
- Head impact: Hood and windshield (child/adult headforms)
- Leg impact: Bumper (lower leg, upper leg)

**Material Considerations**:
- Hood: Aluminum, composites (deformable, energy-absorbing)
- Bumper: Crushable foam, plastic, aluminum
- Space between hood and engine: ≥ 60 mm (energy absorption)

### 6.4 Crashworthiness Optimization

#### 6.4.1 Load Paths

**Principle**: Direct crash loads through strong structures to energy-absorbing zones

**Implementation**:
- Front rails: Aluminum extrusions, AHSS (progressive crush)
- Sill structures: UHSS (load transfer)
- Cross-members: AHSS, aluminum (load distribution)

#### 6.4.2 Material Grading

**Concept**: Tailored material strength in different zones

**Example** (side structure):
- B-pillar upper: UHSS 1,500 MPa (intrusion resistance)
- B-pillar lower: DP 600 MPa (energy absorption)
- Rocker panel: AHSS 980 MPa (load transfer)

#### 6.4.3 Hybrid Structures

**Multi-Material Design**:
- Steel space frame: High-strength, crash protection
- Aluminum/CFRP body panels: Low weight
- Adhesive bonding: Stress distribution, stiffness

**Example**: BMW i3 (CFRP passenger cell + aluminum crash structures)

---

## 7. Corrosion Resistance

### 7.1 Corrosion Mechanisms

#### 7.1.1 Galvanic Corrosion

**Occurs**: When dissimilar metals are in electrical contact in presence of electrolyte

**Galvanic Series** (most noble → least noble in seawater):
1. Graphite (CFRP)
2. Platinum
3. Titanium
4. Stainless Steel (passive)
5. Aluminum Bronze
6. Copper
7. Brass
8. Nickel (passive)
9. Tin
10. Lead
11. Steel / Iron
12. Aluminum Alloys
13. Zinc
14. Magnesium

**Protection Potential Differences** (maximum acceptable):
- Al - Steel: Insulate or coat (ΔE ≈ 0.5V)
- Mg - Al: Insulate or coat (ΔE ≈ 0.7V)
- CFRP - Al: Must insulate (ΔE > 1.0V)

#### 7.1.2 Crevice Corrosion

**Occurs**: In narrow gaps where electrolyte is stagnant (flanges, overlaps, fasteners)

**Prevention**:
- Sealants in joints
- Continuous welds
- Proper drainage

### 7.2 Corrosion Protection Methods

#### 7.2.1 Metallic Coatings

**Galvanizing** (Zinc coating on steel):
- Hot-dip galvanized (HDG): 60-90 μm, Z275-Z350
- Electrogalvanized (EG): 5-15 μm, Z50-Z100
- Galvannealed (GA): 5-15 μm, diffused Fe-Zn
- Protection: Sacrificial (anodic to steel)

**Aluminum Coatings**:
- Hot-dip aluminized (Type 1): 25-40 μm
- Aluminum-silicon (Type 2): 20-30 μm, better formability
- Protection: Barrier + cathodic

#### 7.2.2 Conversion Coatings

**Phosphate Coating** (Steel):
- Zinc phosphate: 2-5 g/m², crystal size 5-20 μm
- Purpose: Paint adhesion, corrosion resistance

**Chromate Conversion Coating** (Aluminum):
- Chromic acid treatment (Alodine)
- Being replaced by chromium-free alternatives (Zr, Ti-based)

**Anodizing** (Aluminum):
- Electrolytic oxidation: Al₂O₃ layer
- Thickness: 5-25 μm (Type II), 25-150 μm (Type III)
- Hard anodizing: Wear resistance

#### 7.2.3 Organic Coatings

**E-coat (Electrodeposition)**:
- Cathodic electrodeposition
- Thickness: 15-25 μm
- Coverage: Excellent (cavities, edges)
- Cure: 170-180°C, 20-30 minutes

**Powder Coating**:
- Epoxy, polyester, polyurethane
- Thickness: 60-100 μm
- Application: Electrostatic spray

**Liquid Paint**:
- Primer: 15-25 μm
- Base coat: 15-25 μm
- Clear coat: 35-50 μm

#### 7.2.4 Barrier and Insulation

**For Multi-Material Joints**:
- Glass fiber tape: Electrical insulation
- Adhesive interlayers: Barrier + bonding
- Rubber gaskets: Sealing + insulation
- Plastic washers: Insulation at fasteners

### 7.3 Testing Standards

#### 7.3.1 Salt Spray Test

**ASTM B117 / ISO 9227**:
- Solution: 5% NaCl, pH 6.5-7.2
- Temperature: 35°C
- Duration: 240-1,000 hours (depending on requirement)

**Acceptance Criteria**:
- Scribe creep: < 2-4 mm from scribe
- Blistering: None or size ≤ 2 (ASTM D714)

#### 7.3.2 Cyclic Corrosion Test

**VDA 621-415 / SAE J2334**:
- Salt spray + humidity + dry cycles
- Temperature cycling
- More realistic than continuous salt spray
- Duration: 10-30 cycles (1 cycle = 24 hours)

#### 7.3.3 Accelerated Tests

**GM 9540P** (Cosmetic Corrosion):
- 15 days accelerated exposure
- Equivalent to ~10 years field exposure

---

## 8. Recyclability

### 8.1 Material Recyclability

#### 8.1.1 Ferrous Metals

**Steel**:
- Recycling rate: 85-90% (vehicles)
- Process: Shredding → magnetic separation → melting
- Secondary material quality: Excellent (infinite recyclability)
- Energy savings: 60-75% vs. primary production

#### 8.1.2 Non-Ferrous Metals

**Aluminum**:
- Recycling rate: 90-95%
- Process: Shredding → eddy current separation → melting
- Energy savings: 95% vs. primary production
- Challenges: Alloy sorting, coating removal

**Magnesium**:
- Recycling rate: 30-40% (improving)
- Process: Melting with flux
- Challenges: Oxidation, alloy contamination

#### 8.1.3 Composites

**Thermoset Composites** (CFRP, GFRP with epoxy):
- Current recycling rate: 10-30%
- Methods:
  1. Mechanical recycling: Grinding → filler material (low value)
  2. Thermal recycling: Pyrolysis (450-700°C) → reclaimed fibers (60-80% strength)
  3. Chemical recycling: Solvolysis → reclaimed fibers + resin monomers
- Challenges: Fiber length reduction, contamination, cost

**Thermoplastic Composites**:
- Recycling rate: 40-60%
- Process: Remelting and remolding
- Properties: Reduced after multiple cycles

### 8.2 Design for Recyclability

#### 8.2.1 Material Selection

**Guidelines**:
1. Minimize number of different materials
2. Use recyclable materials where possible
3. Avoid hazardous substances (RoHS, REACH compliance)
4. Use thermoplastics instead of thermosets when feasible

#### 8.2.2 Joining Methods

**Preference**:
1. Mechanical fasteners (disassembly-friendly)
2. Adhesives (compatible with recycling)
3. Welding (if materials are same type)

**Avoid**: Difficult-to-separate joints (mixed-material adhesive bonds)

#### 8.2.3 Material Marking

**ISO 1043-1** (Plastics):
- Mark all plastic parts > 25 grams
- Location: Flat surface, easily visible after disassembly
- Size: ≥ 3 mm height

**ISO 1629** (Elastomers):
- Similar marking requirement

### 8.3 End-of-Life Vehicle (ELV) Directive

**EU Directive 2000/53/EC**:
- Reuse + Recovery target: 95% by weight (2015+)
- Reuse + Recycling target: 85% by weight (2015+)
- Hazardous substances restrictions: Pb, Hg, Cd, Cr(VI)

**Material Recovery Targets**:
- Metals: 95%+ (ferrous, aluminum)
- Plastics: 25% (increasing)
- Glass: 80%
- Fluids: 100% (oil, coolant, fuel)
- Tires: 100% (material recycling or energy recovery)

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

## 10. API Interface

### 10.1 Material Database API

#### 10.1.1 Get Material Properties

```typescript
interface MaterialRequest {
  materialId: string;
  properties?: string[]; // e.g., ['mechanical', 'thermal']
  temperature?: number; // °C (for temperature-dependent properties)
}

interface MaterialResponse {
  material: {
    id: string;
    name: string;
    category: string;
    properties: {
      mechanical?: MechanicalProperties;
      thermal?: ThermalProperties;
      electrical?: ElectricalProperties;
    };
  };
  metadata: {
    source: string;
    testStandards: string[];
    lastUpdated: string;
  };
}
```

#### 10.1.2 Compare Materials

```typescript
interface ComparisonRequest {
  materialIds: string[];
  metrics: string[]; // e.g., ['specificStrength', 'cost', 'recyclability']
  weightingFactors?: Record<string, number>; // For scoring
}

interface ComparisonResponse {
  materials: MaterialSummary[];
  comparison: {
    metric: string;
    values: number[];
    normalized: number[]; // 0-1 scale
  }[];
  recommendation: {
    materialId: string;
    score: number;
    reasoning: string;
  };
}
```

### 10.2 Weight Reduction Calculator API

```typescript
interface WeightReductionRequest {
  component: string;
  originalMaterial: string;
  newMaterial: string;
  geometry: {
    type: 'sheet' | 'tube' | 'extrusion' | 'casting' | 'custom';
    dimensions: Record<string, number>;
  };
  quantity?: number; // For total vehicle calculation
}

interface WeightReductionResponse {
  weightSaved: number; // kg per component
  totalWeightSaved: number; // kg for all components
  percentReduction: number;
  fuelSavings: {
    percent: number;
    liters100km: number;
    co2Reduction: number; // g/km
  };
  costImplication: {
    materialCost: number;
    toolingCost: number;
    totalCost: number;
    paybackPeriod: number; // years
  };
}
```

### 10.3 Material Selection API

```typescript
interface SelectionCriteria {
  application: string;
  loadCases: LoadCase[];
  constraints: {
    maxWeight?: number;
    maxCost?: number;
    minStrength?: number;
    minStiffness?: number;
    corrosionResistance?: 'low' | 'medium' | 'high';
    formability?: 'low' | 'medium' | 'high';
  };
  preferences?: {
    recyclability?: number; // 0-1
    sustainability?: number; // 0-1
    availability?: number; // 0-1
  };
}

interface MaterialRecommendation {
  ranking: {
    materialId: string;
    score: number;
    pros: string[];
    cons: string[];
  }[];
  analysis: {
    weightVsStrength: ChartData;
    costVsPerformance: ChartData;
    ashbyChart: ChartData;
  };
}
```

### 10.4 Crash Simulation API

```typescript
interface CrashSimulationRequest {
  material: string;
  geometry: {
    type: 'tube' | 'beam' | 'panel';
    dimensions: Record<string, number>;
    wallThickness: number;
  };
  impactConditions: {
    velocity: number; // m/s
    mass: number; // kg
    angle: number; // degrees
  };
  constraints: 'fixed' | 'simply-supported' | 'free';
}

interface CrashSimulationResponse {
  energyAbsorbed: number; // J
  peakForce: number; // N
  meanForce: number; // N
  crushDistance: number; // mm
  specificEnergyAbsorption: number; // kJ/kg
  deformationMode: string;
  timeline: {
    time: number[]; // ms
    force: number[]; // N
    displacement: number[]; // mm
  };
}
```

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

## 12. References

### 12.1 Standards Organizations

- **ISO**: International Organization for Standardization
- **ASTM**: American Society for Testing and Materials
- **SAE**: Society of Automotive Engineers
- **VDA**: German Association of the Automotive Industry
- **NCAP**: New Car Assessment Program (Euro NCAP, US NCAP, China NCAP)
- **FMVSS**: Federal Motor Vehicle Safety Standards (USA)

### 12.2 Key Publications

1. Ashby, M. F. (2011). *Materials Selection in Mechanical Design* (4th ed.). Butterworth-Heinemann.
2. Hirsch, J. (2011). "Aluminum in Innovative Light-Weight Car Design." *Materials Transactions*, 52(5), 818-824.
3. Czerwinski, F. (2021). *Current Trends in Automotive Lightweighting Strategies and Materials*. Materials, 14(21), 6631.
4. 선행 연구. "Innovative and Highly Productive Joining Technologies for Multi-Material Lightweight Car Body Structures." *Journal of Materials Engineering and Performance*, 23(5), 1515-1523.
5. Elmarakbi, A. (2014). *Advanced Composite Materials for Automotive Applications: Structural Integrity and Crashworthiness*. Wiley.

### 12.3 Industry Reports

- *Aluminum in Cars* - European Aluminium Association (2022)
- *The Future of Automotive Lightweighting* - McKinsey & Company (2023)
- *Carbon Fiber Composites Market Report* - Lucintel (2024)
- *Advanced Materials in Automotive Engineering* - SAE International (2023)

### 12.4 Related WIA Standards

- **WIA-AUTO-001 to WIA-AUTO-020**: Other automotive standards
- **WIA-SUSTAINABILITY**: Lifecycle assessment and environmental impact
- **WIA-MANUFACTURING**: Advanced manufacturing processes
- **WIA-SAFETY**: Safety and crash protection systems
- **WIA-MATERIALS**: General materials engineering standards

---

## Appendix A: Material Selection Charts

### A.1 Ashby Chart - Strength vs. Density

```
           |
1000   CFRP|                 ● CFRP
           |              ●
           |          ● Titanium
Strength   |       ● AHSS/UHSS
(MPa)      |     ● Al 7075
           |   ● Al 6061
100        | ● Mg alloys
           |● Steel
           |___________________
           1000  2000  5000  10000
                 Density (kg/m³)
```

### A.2 Specific Strength Comparison

```
CFRP (Standard):    |████████████████████████████████| 500
CFRP (High Modulus):|███████████████████████████████████| 625
Al 7075-T6:         |█████| 204
Titanium Ti-6Al-4V: |█████| 214
AHSS (DP 1000):     |███| 127
Mg AZ31B:           |███| 147
Al 6061-T6:         |███| 115
UHSS (1500 MPa):    |████| 191
Steel (Mild):       |█| 51
                    0   100  200  300  400  500  600
                    Specific Strength (kN·m/kg)
```

### A.3 Cost vs. Weight Savings

```
Material          | Cost Factor | Weight Savings | Cost-Performance
------------------|-------------|----------------|------------------
Steel (baseline)  | 1.0         | 0%             | 1.0
HSS               | 1.2         | 10-15%         | 0.9
AHSS              | 1.5         | 15-25%         | 0.8
Aluminum          | 2.5-4.0     | 40-50%         | 0.7
Magnesium         | 3.5-5.0     | 60-70%         | 0.6
CFRP              | 15-30       | 50-60%         | 0.3
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-021 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
