# WIA-AUTO-021 — Phase 4: Integration

> Vehicle-lightweight-material canonical Phase 4: ecosystem integration (AEC + ISO + EU ELV + Battery Reg + IATF 16949 + LCA + future).

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



---

## A.1 Standards cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| Tensile testing — metals      | ASTM E8 / ISO 6892                        |
| Tensile testing — plastics    | ASTM D638 / ISO 527                       |
| Tensile testing — composites  | ASTM D3039 / ISO 527-5                    |
| Charpy impact                 | ASTM E23 / ISO 148                        |
| Fatigue-crack growth          | ASTM E647 / ISO 12108                     |
| Fracture toughness K_IC       | ASTM E1820 / ISO 12135                    |
| Welding — automotive          | AWS D8.7 / AWS D8.9                       |
| Adhesive lap-shear            | ASTM D1002 / ISO 4587                     |
| Cyclic corrosion              | SAE J2334 / VDA 233-102 / GMW14872        |
| Galvanic-corrosion mitigation | SAE J2747                                 |
| Strain-rate dependence        | SAE J2749                                 |
| Recyclability calculation     | ISO 22628                                 |
| End-of-Life Vehicles          | EU Directive 2000/53/EC                   |
| Battery Regulation            | EU Regulation 2023/1542                   |
| Vehicle classification        | UN ECE R29 / R94 / R95 / FMVSS 208/214/301|
| Consumer crash rating         | Euro NCAP / IIHS / KNCAP / JNCAP          |
| Calibration / lab competence  | ISO/IEC 17025                             |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 OEM and supplier integration

OEM integration captures the BIW (body-in-white) material-mix envelope per the OEM's lightweighting strategy, the supplier-of-record envelope per Tier-1 / Tier-2 / Tier-3 mapping, the IATF 16949 quality-management-system envelope for the supplier's plant of origin, the PPAP (Production Part Approval Process) envelope per AIAG, and the per-program FMEA reference (DFMEA + PFMEA per AIAG-VDA harmonised method). Material substitution during a program follows a documented design-change-notification (DCN) envelope with re-validation gates for crash, fatigue, and corrosion.

## A.3 Sustainability and life-cycle integration

Sustainability integration captures the cradle-to-gate carbon footprint per ISO 14040 / ISO 14044 LCA, the recyclate content commitment, the embodied-energy envelope per material grade, the per-component life-cycle inventory, and the end-of-life material-recovery commitment. Reporting follows the ISO 14026 Footprint Communication framework for B2B disclosures and the GHG Protocol Corporate Standard for the OEM's Scope 3 reporting per category 1 (purchased goods and services) and category 11 (use of sold products).

## A.4 Repair and aftermarket integration

Repair integration honours the OEM's repair-procedure envelope: per-component repair vs. replace decisions, the structural-repair envelope (no field welding on press-hardened steel B-pillars; no cold-work on TRIP / TWIP grades; specified adhesive systems for high-strength steel partial repairs), and the body-shop certification envelope. Repair-procedure documents reference the OEM's published repair manuals and the I-CAR / Thatcham equivalent industry curricula. Aftermarket replacement parts must meet the same material-class envelope as the original equipment for crash performance to remain equivalent.

## A.5 Future directions

Active research tracks: third-generation advanced high-strength steels (3rd-Gen AHSS) at 1500-2000 MPa with 20-30% elongation; magnesium alloys with improved ignition resistance for fire-safety; thermoplastic composite chassis modules with rapid molding cycle; aluminium-steel hybrid joints with consistent corrosion-class envelope; carbon-fibre recycling at industrial scale; bio-composite interior-trim with documented life-cycle reduction; multi-material additive manufacturing for low-volume / high-mix structural castings (Tesla-style mega-castings). The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- ASTM E8 / E23 / E647 / E1820 — mechanical and fracture testing
- ISO 6892 / 148 / 12108 / 12135 — equivalents
- ASTM D3039 / ISO 527 — composite tensile
- ASTM D1002 / ISO 4587 — adhesive lap-shear
- AWS D8.7 / D8.9 — automotive welding
- SAE J2334 — laboratory corrosion
- SAE J2747 — galvanic-corrosion mitigation in vehicle joints
- SAE J2749 — strain-rate dependence
- VDA 233-102 — cyclic corrosion
- ISO 22628 — recyclability and recoverability calculation
- EU Directive 2000/53/EC — End-of-Life Vehicles
- EU Regulation 2023/1542 — Batteries Regulation
- ISO 14040 / 14044 — LCA principles and requirements
- IATF 16949 — automotive quality management
- AIAG-VDA FMEA Handbook (2019)
- Euro NCAP / IIHS / KNCAP / JNCAP / Latin NCAP / ASEAN NCAP — consumer ratings
- FMVSS 208 / 214 / 301 — US crash standards
- UN ECE R29 / R94 / R95 — frontal / side / rear crash regulations


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
