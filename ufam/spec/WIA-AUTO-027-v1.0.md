# WIA-AUTO-027: Universal Fluid for All Mobility Specification v1.0

> **Standard ID:** WIA-AUTO-027
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Fluids Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Universal Fluid Chemistry](#2-universal-fluid-chemistry)
3. [Multi-Functional Applications](#3-multi-functional-applications)
4. [Bio-Based and Sustainable Fluids](#4-bio-based-and-sustainable-fluids)
5. [Smart Fluid Monitoring](#5-smart-fluid-monitoring)
6. [Condition-Based Fluid Management](#6-condition-based-fluid-management)
7. [Compatibility Requirements](#7-compatibility-requirements)
8. [Environmental Impact](#8-environmental-impact)
9. [Data Formats](#9-data-formats)
10. [API Interface](#10-api-interface)
11. [Testing Standards](#11-testing-standards)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the requirements, testing methods, and implementation guidelines for universal fluid systems in modern mobility applications. The standard aims to reduce the complexity of fluid management, improve environmental sustainability, and enable intelligent condition-based maintenance.

### 1.2 Scope

The standard covers:
- Chemical composition and properties of universal fluids
- Multi-functional fluid applications (cooling, lubrication, hydraulics)
- Bio-based and synthetic fluid formulations
- Smart monitoring systems and sensors
- Predictive maintenance algorithms
- Environmental impact and sustainability metrics
- API specifications for fluid management systems

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard promotes environmental sustainability, reduced waste, improved efficiency, and simplified maintenance for all types of mobility systems, from personal vehicles to heavy machinery and marine vessels.

### 1.4 Terminology

- **Universal Fluid**: A multi-purpose fluid designed to perform multiple functions (cooling, lubrication, hydraulic power transmission)
- **Viscosity Index (VI)**: A measure of fluid viscosity change with temperature
- **Total Acid Number (TAN)**: Measure of acidity in fluid (mg KOH/g)
- **Total Base Number (TBN)**: Measure of alkalinity in fluid (mg KOH/g)
- **Kinematic Viscosity**: Fluid resistance to flow under gravity (cSt)
- **Pour Point**: Lowest temperature at which fluid flows (°C)
- **Flash Point**: Lowest temperature at which fluid vapor ignites (°C)
- **Biodegradability**: Percentage breakdown by microorganisms in specified time

---

## 2. Universal Fluid Chemistry

### 2.1 Base Fluid Types

#### 2.1.1 Mineral Oil Base
Traditional petroleum-derived base fluids:
```
Composition: Paraffinic or naphthenic hydrocarbons
Viscosity Index: 95-110
Cost Factor: 1.0× (baseline)
Biodegradability: <20% in 28 days
```

#### 2.1.2 Synthetic Base
Chemically synthesized base fluids:
```
Types: PAO, Esters, Polyglycols, Silicones
Viscosity Index: 130-200
Cost Factor: 2.5-4.0×
Biodegradability: 30-90% depending on type
```

#### 2.1.3 Bio-Based Base
Plant-derived renewable base fluids:
```
Sources: Vegetable oils (rapeseed, soy, sunflower), algae
Viscosity Index: 150-180
Cost Factor: 1.5-2.5×
Biodegradability: >90% in 28 days
```

### 2.2 Additive Packages

Universal fluids require balanced additive packages:

#### 2.2.1 Viscosity Modifiers
```
Purpose: Maintain viscosity across temperature range
Types: Polymethacrylates (PMA), Olefin copolymers (OCP)
Concentration: 5-15% by weight
Effect: Increases VI by 40-100 points
```

#### 2.2.2 Anti-Wear Additives
```
Purpose: Reduce friction and wear in lubricated contacts
Types: ZDDP, TCP, friction modifiers
Concentration: 0.5-2% by weight
Mechanism: Form protective tribofilm on metal surfaces
```

#### 2.2.3 Antioxidants
```
Purpose: Prevent fluid oxidation and degradation
Types: Phenolic, aminic, sulfur-phosphorus compounds
Concentration: 0.5-3% by weight
Mechanism: Chain-breaking and peroxide decomposition
```

#### 2.2.4 Corrosion Inhibitors
```
Purpose: Protect metal surfaces from corrosion
Types: Carboxylic acids, amines, sulfonates
Concentration: 0.1-0.5% by weight
Application: Forms protective barrier on metal surfaces
```

#### 2.2.5 Pour Point Depressants
```
Purpose: Lower fluid pour point for cold weather operation
Types: Polymethacrylates, alkylated naphthalenes
Concentration: 0.1-1% by weight
Effect: Reduces pour point by 10-30°C
```

### 2.3 Physical Properties Requirements

#### 2.3.1 Viscosity Specifications

**Kinematic Viscosity** (ASTM D445):
```
At 40°C:  40-55 cSt
At 100°C: 7.5-10 cSt
Viscosity Index (VI): ≥140
```

Viscosity-temperature relationship:
```
log(log(ν + 0.8)) = A - B × log(T)
```
Where:
- `ν` = Kinematic viscosity (cSt)
- `T` = Absolute temperature (K)
- `A, B` = Fluid-specific constants

#### 2.3.2 Thermal Properties

**Pour Point** (ASTM D97):
```
Universal Fluid Grade A: ≤ -35°C
Universal Fluid Grade B: ≤ -25°C
Universal Fluid Grade C: ≤ -15°C
```

**Flash Point** (ASTM D92):
```
Minimum: 200°C
Recommended: ≥220°C
```

**Thermal Conductivity**:
```
At 25°C: 0.13-0.16 W/m·K
At 100°C: 0.11-0.14 W/m·K
```

**Specific Heat Capacity**:
```
At 25°C: 1.8-2.2 kJ/kg·K
At 100°C: 2.0-2.5 kJ/kg·K
```

#### 2.3.3 Density and Compressibility

**Density** (ASTM D4052):
```
At 15°C: 850-900 kg/m³
At 100°C: 800-850 kg/m³
```

**Bulk Modulus** (compressibility):
```
At 40°C and 70 MPa: ≥1.5 GPa
Temperature coefficient: -0.5 to -1.0 GPa/100°C
```

---

## 3. Multi-Functional Applications

### 3.1 Cooling Function

#### 3.1.1 Heat Transfer Performance

Heat transfer rate in cooling systems:
```
Q = ṁ × cp × ΔT
```
Where:
- `Q` = Heat transfer rate (W)
- `ṁ` = Mass flow rate (kg/s)
- `cp` = Specific heat capacity (J/kg·K)
- `ΔT` = Temperature difference (K)

Convective heat transfer:
```
Q = h × A × (Ts - Tf)
```
Where:
- `h` = Convective heat transfer coefficient (W/m²·K)
- `A` = Surface area (m²)
- `Ts` = Surface temperature (K)
- `Tf` = Fluid temperature (K)

**Heat Transfer Coefficient Requirements**:
```
Laminar flow (Re < 2300): h ≥ 500 W/m²·K
Turbulent flow (Re > 4000): h ≥ 2000 W/m²·K
```

#### 3.1.2 Thermal Stability

**Maximum Operating Temperature**:
```
Continuous: 150°C
Peak (short duration): 180°C
Flash heating: 200°C
```

**Oxidation Stability** (ASTM D943):
```
Minimum oxidation life: 2000 hours at 95°C
TAN increase: <2.0 mg KOH/g per 1000 hours
Viscosity change: <15% over service life
```

### 3.2 Lubrication Function

#### 3.2.1 Film Thickness and Bearing Protection

Minimum film thickness in hydrodynamic lubrication:
```
hmin = (2.65 × μ × U × R) / (W × (1 - 0.58 × e²)^0.5)
```
Where:
- `hmin` = Minimum film thickness (m)
- `μ` = Dynamic viscosity (Pa·s)
- `U` = Surface velocity (m/s)
- `R` = Bearing radius (m)
- `W` = Load per unit length (N/m)
- `e` = Eccentricity ratio

**Lubrication Requirements**:
```
Film thickness ratio (λ): ≥3 for full film lubrication
Coefficient of friction: 0.001-0.005 (hydrodynamic)
Wear rate: <0.1 μm/1000 hours
```

#### 3.2.2 Extreme Pressure Properties

**Four-Ball EP Test** (ASTM D2783):
```
Weld point: ≥250 kg
Load wear index (LWI): ≥40
```

**Timken OK Load** (ASTM D2782):
```
Minimum: 30 lb
Recommended: ≥40 lb
```

#### 3.2.3 Anti-Wear Performance

**Four-Ball Wear Test** (ASTM D4172):
```
Test conditions: 1200 rpm, 75°C, 1 hour, 40 kg load
Wear scar diameter: ≤0.45 mm
```

### 3.3 Hydraulic Function

#### 3.3.1 Hydraulic System Performance

**Pressure-Flow Relationship**:
```
Q = (π × d⁴ × ΔP) / (128 × μ × L)
```
Where:
- `Q` = Volumetric flow rate (m³/s)
- `d` = Pipe diameter (m)
- `ΔP` = Pressure drop (Pa)
- `μ` = Dynamic viscosity (Pa·s)
- `L` = Pipe length (m)

**Hydraulic System Requirements**:
```
Operating pressure: Up to 35 MPa (350 bar)
Peak pressure: 42 MPa (420 bar)
Response time: <50 ms for pressure change
Bulk modulus: ≥1.5 GPa at operating temperature
```

#### 3.3.2 Filterability and Cleanliness

**ISO Cleanliness Code** (ISO 4406):
```
Target: 16/14/11 or better
Critical systems: 15/13/10 or better
```

**Filter Performance**:
```
Filtration rating: β25 ≥ 200
(99.5% removal of particles ≥25 μm)
```

#### 3.3.3 Air Release and Foam Resistance

**Air Release** (ASTM D3427):
```
At 50°C: ≤10 minutes to 0.2% air
```

**Foam Test** (ASTM D892):
```
Sequence I (24°C):  ≤50 mL foam, 0 mL after settling
Sequence II (94°C): ≤50 mL foam, 0 mL after settling
Sequence III (24°C): ≤50 mL foam, 0 mL after settling
```

---

## 4. Bio-Based and Sustainable Fluids

### 4.1 Biodegradability Requirements

#### 4.1.1 Test Methods

**OECD 301B** (CO₂ Evolution Test):
```
Pass criteria: ≥60% biodegradation in 28 days
Ultimate biodegradability: ≥90% in 60 days
```

**Biodegradation Rate Model**:
```
B(t) = Bmax × (1 - e^(-k×t))
```
Where:
- `B(t)` = Biodegradation at time t (%)
- `Bmax` = Maximum biodegradation (%)
- `k` = Rate constant (day⁻¹)
- `t` = Time (days)

#### 4.1.2 Bio-Based Content

**ASTM D6866** (Radiocarbon Analysis):
```
Bio-based content categories:
- Grade I: ≥90% bio-based carbon
- Grade II: ≥70% bio-based carbon
- Grade III: ≥50% bio-based carbon
```

### 4.2 Environmental Toxicity

#### 4.2.1 Aquatic Toxicity

**LC50 Test** (OECD 203 - Fish Acute Toxicity):
```
Required: LC50 > 100 mg/L (96 hours)
Preferred: LC50 > 1000 mg/L (practically non-toxic)
```

**EC50 Test** (OECD 202 - Daphnia Acute Toxicity):
```
Required: EC50 > 100 mg/L (48 hours)
```

#### 4.2.2 Bioaccumulation

**Log Kow** (Octanol-Water Partition Coefficient):
```
Recommended: Log Kow < 4.5
(Low bioaccumulation potential)
```

### 4.3 Renewable Content Calculation

Carbon footprint comparison:
```
CF = (E_production + E_transport + E_disposal) × CO₂_factor
```
Where:
- `CF` = Carbon footprint (kg CO₂e)
- `E_production` = Production energy (MJ)
- `E_transport` = Transportation energy (MJ)
- `E_disposal` = Disposal energy (MJ)
- `CO₂_factor` = Emission factor (kg CO₂e/MJ)

**Target Reduction**:
```
Bio-based fluids: 30-60% lower CF than mineral oil
Recycled fluids: 40-70% lower CF than virgin fluids
```

---

## 5. Smart Fluid Monitoring

### 5.1 Sensor Technologies

#### 5.1.1 Viscosity Sensors

**Micro-Electro-Mechanical Systems (MEMS)**:
```
Technology: Vibrating element or pressure drop measurement
Range: 10-200 cSt
Accuracy: ±2% of reading
Resolution: 0.1 cSt
Response time: <5 seconds
```

**Viscosity Temperature Compensation**:
```
ν(T) = ν₀ × e^(B/(T-C))
```
Where:
- `ν(T)` = Viscosity at temperature T (cSt)
- `ν₀` = Reference viscosity (cSt)
- `B, C` = Fluid-specific constants
- `T` = Temperature (K)

#### 5.1.2 Temperature Sensors

**Thermocouple or RTD**:
```
Range: -40°C to +200°C
Accuracy: ±0.5°C
Resolution: 0.1°C
Response time: <2 seconds
```

#### 5.1.3 Contamination Sensors

**Particle Counter**:
```
Method: Optical extinction or light scattering
Size range: 4-100 μm
ISO 4406 reporting: Yes
Accuracy: ±10% of reading
```

**Water-in-Oil Sensor**:
```
Technology: Capacitance or dielectric constant
Range: 0-5% water content
Accuracy: ±0.05% absolute
Resolution: 0.01%
```

#### 5.1.4 Chemical Degradation Sensors

**Dielectric Constant Sensor**:
```
Frequency: 1-10 MHz
Range: Relative permittivity 1.5-3.5
Correlation: TAN, oxidation, contamination
```

**Infrared Spectroscopy** (optional):
```
Method: FTIR with ATR crystal
Parameters: Oxidation, nitration, sulfation
Wavelengths: 1700-1750 cm⁻¹ (oxidation)
```

### 5.2 Data Acquisition and Processing

#### 5.2.1 Sampling Rate

```
Continuous monitoring: 1 sample/minute
High-load conditions: 1 sample/second
Fleet monitoring: 1 sample/hour
```

#### 5.2.2 Data Format

**JSON Structure**:
```json
{
  "timestamp": "2025-12-26T10:30:00Z",
  "sensor_id": "UFAM-001",
  "vehicle_id": "VEH-12345",
  "fluid_type": "universal-synthetic",
  "measurements": {
    "viscosity": {
      "value": 45.2,
      "unit": "cSt",
      "temperature": 40
    },
    "tan": {
      "value": 1.8,
      "unit": "mg_KOH_per_g"
    },
    "water_content": {
      "value": 0.05,
      "unit": "percent"
    },
    "particle_count": {
      "value": 15,
      "unit": "particles_per_mL",
      "size_threshold": 25
    },
    "temperature": {
      "value": 85,
      "unit": "celsius"
    }
  }
}
```

### 5.3 Predictive Analytics

#### 5.3.1 Remaining Useful Life (RUL) Estimation

**Linear Degradation Model**:
```
RUL = (TAN_limit - TAN_current) / (dTAN/dt)
```
Where:
- `TAN_limit` = Condemning limit (typically 4.0 mg KOH/g)
- `TAN_current` = Current TAN value
- `dTAN/dt` = Rate of TAN increase

**Machine Learning Model**:
```
RUL = f(viscosity, TAN, TBN, water, particles, T, operating_hours)
```
Using algorithms:
- Random Forest Regression
- Gradient Boosting
- Neural Networks

#### 5.3.2 Anomaly Detection

**Statistical Process Control**:
```
UCL = μ + 3σ (Upper Control Limit)
LCL = μ - 3σ (Lower Control Limit)
```
Where:
- `μ` = Mean value
- `σ` = Standard deviation

**Multivariate Analysis**:
```
Mahalanobis Distance: D² = (x - μ)ᵀ Σ⁻¹ (x - μ)
```
Where:
- `x` = Current measurement vector
- `μ` = Mean vector
- `Σ` = Covariance matrix

---

## 6. Condition-Based Fluid Management

### 6.1 Fluid Condition Indicators

#### 6.1.1 Primary Indicators

**Viscosity Change**:
```
Condemning limit: ±15% from new fluid viscosity
Action limit: ±10% triggers investigation
```

**Total Acid Number (TAN)**:
```
New fluid: <1.0 mg KOH/g
Action limit: 2.5 mg KOH/g
Condemning limit: 4.0 mg KOH/g
```

**Water Content**:
```
Normal: <0.05%
Action limit: 0.1%
Condemning limit: 0.2%
```

**Particle Contamination** (ISO 4406):
```
Normal: 16/14/11 or better
Action: 18/16/13
Condemning: 20/18/15
```

#### 6.1.2 Secondary Indicators

**Total Base Number (TBN)** (for fluids with alkaline reserve):
```
New fluid: Varies by formulation
Condemning limit: <50% of new fluid TBN
```

**Oxidation** (FTIR peak area):
```
Normal: <10 abs/cm
Action: 10-20 abs/cm
Condemning: >20 abs/cm
```

**Nitration** (FTIR peak area):
```
Normal: <15 abs/cm
Action: 15-25 abs/cm
Condemning: >25 abs/cm
```

### 6.2 Replacement Decision Algorithm

#### 6.2.1 Multi-Criteria Decision Matrix

```python
def calculate_fluid_health_score(measurements):
    """
    Calculate composite health score (0-100)
    100 = new fluid, 0 = condemning limit
    """
    weights = {
        'viscosity': 0.25,
        'tan': 0.30,
        'water': 0.20,
        'particles': 0.15,
        'oxidation': 0.10
    }

    scores = {}

    # Viscosity score
    visc_change = abs(measurements['viscosity'] - reference['viscosity'])
    scores['viscosity'] = max(0, 100 - (visc_change / 0.15) * 100)

    # TAN score
    scores['tan'] = max(0, 100 - (measurements['tan'] / 4.0) * 100)

    # Water score
    scores['water'] = max(0, 100 - (measurements['water'] / 0.002) * 100)

    # Particle score
    scores['particles'] = iso_code_to_score(measurements['iso_code'])

    # Oxidation score
    scores['oxidation'] = max(0, 100 - (measurements['oxidation'] / 20) * 100)

    # Weighted average
    health_score = sum(scores[k] * weights[k] for k in weights)

    return health_score, scores
```

#### 6.2.2 Replacement Decision Tree

```
IF health_score ≥ 80 THEN
    Action = "Continue monitoring"
    Interval = "Normal (monthly)"

ELSE IF health_score ≥ 60 THEN
    Action = "Increase monitoring"
    Interval = "Frequent (weekly)"

ELSE IF health_score ≥ 40 THEN
    Action = "Plan replacement"
    Timeline = "Within 1 month"

ELSE
    Action = "Immediate replacement"
    Priority = "Urgent"
END IF
```

### 6.3 Life Extension Strategies

#### 6.3.1 Filtration Enhancement

**Kidney Loop Filtration**:
```
Flow rate: 10-20% of system capacity
Filter rating: β25 ≥ 200
Continuous operation during idle periods
```

**Expected Benefits**:
```
Life extension: 50-100%
Contamination reduction: 2-3 ISO code levels
```

#### 6.3.2 Fluid Reclamation

**Vacuum Dehydration**:
```
Water removal: 0.2% → 0.05%
Air removal: 10% → 0.2%
Temperature: 60-80°C
Vacuum: 50-100 mbar
```

**Chemical Replenishment**:
```
Additive boost: Restore depleted additives
TBN restoration: Add alkaline additives
Antioxidant addition: Extend oxidation life
```

---

## 7. Compatibility Requirements

### 7.1 Seal and Elastomer Compatibility

#### 7.1.1 Acceptable Materials

**Nitrile Rubber (NBR)**:
```
Volume swell: 0-15%
Hardness change: ±5 Shore A
Tensile strength retention: ≥90%
Test duration: 168 hours at 100°C
```

**Fluorocarbon (FKM)**:
```
Volume swell: 0-10%
Hardness change: ±5 Shore A
Tensile strength retention: ≥95%
Test duration: 168 hours at 150°C
```

**Polytetrafluoroethylene (PTFE)**:
```
Volume swell: 0-5%
Chemical resistance: Excellent
Temperature range: -40°C to +200°C
```

#### 7.1.2 Testing Method

**ASTM D471** (Rubber Property—Effect of Liquids):
```
Test temperature: Service temperature + 20°C
Duration: 168 hours minimum
Measurements: Volume, hardness, tensile strength
```

### 7.2 Metal Compatibility

#### 7.2.1 Corrosion Protection

**Copper Strip Corrosion** (ASTM D130):
```
Rating: Maximum 1b at 100°C for 3 hours
```

**Rust Prevention** (ASTM D665):
```
Procedure A (distilled water): Pass (no rust)
Procedure B (synthetic seawater): Pass (no rust)
```

#### 7.2.2 Metal Compatibility Matrix

| Metal | Compatibility | Protective Measures |
|-------|---------------|---------------------|
| Steel | Excellent | Standard corrosion inhibitors |
| Aluminum | Good | Chelating agents, pH control |
| Copper/Brass | Good | Benzotriazole additives |
| Bronze | Excellent | Standard formulation |
| Stainless Steel | Excellent | No special requirements |
| Magnesium | Fair | Specialized inhibitors required |

### 7.3 Paint and Coating Compatibility

**Paint Compatibility Test**:
```
Method: ASTM D1308
Duration: 24 hours at 80°C
Result: No blistering, softening, or color change
```

### 7.4 Cross-Compatibility with Existing Fluids

#### 7.4.1 Mixing Compatibility

Universal fluids must be compatible with:
```
- Mineral hydraulic oils (ISO VG 32-68)
- ATF (Automatic Transmission Fluid)
- Engine coolant (ethylene glycol based)
- Brake fluid DOT 3/4 (in separate systems)
```

**Mixing Test Protocol**:
```
Ratios: 10%, 25%, 50%, 75%, 90%
Tests: Viscosity, TAN, compatibility
Duration: 72 hours at 80°C
Pass criteria: No phase separation, precipitation, or property degradation
```

---

## 8. Environmental Impact

### 8.1 Life Cycle Assessment (LCA)

#### 8.1.1 Production Phase

**Energy Consumption**:
```
Mineral base: 15-20 MJ/kg
Synthetic base: 40-60 MJ/kg
Bio-based: 10-15 MJ/kg
```

**CO₂ Emissions**:
```
Mineral base: 1.2-1.5 kg CO₂e/kg fluid
Synthetic base: 3.0-4.5 kg CO₂e/kg fluid
Bio-based: 0.5-1.0 kg CO₂e/kg fluid
```

#### 8.1.2 Use Phase

**Efficiency Impact**:
```
Reduced friction: 2-5% fuel/energy savings
Optimized viscosity: 1-3% energy savings
Extended drain intervals: Reduced waste by 50-70%
```

#### 8.1.3 End-of-Life Phase

**Disposal Options**:
```
1. Re-refining: 80-90% recovery, 0.3 kg CO₂e/kg
2. Energy recovery: 40 MJ/kg, 1.0 kg CO₂e/kg
3. Landfill: Not recommended, 2.5 kg CO₂e/kg
4. Biodegradation: For bio-based fluids, 0.1 kg CO₂e/kg
```

### 8.2 Eco-Toxicity Requirements

#### 8.2.1 Labeling Requirements

**EU Regulation (EC) No 1272/2008** (CLP):
```
Target: No hazard pictograms
Maximum: Warning symbol only
Prohibited: Danger symbol, environmental hazard symbol
```

#### 8.2.2 Biodegradation Standards

**Readily Biodegradable** (OECD 301):
```
Required: ≥60% biodegradation in 28 days
Test methods: 301B (CO₂ evolution) or 301F (respirometry)
```

**Inherently Biodegradable** (OECD 302):
```
Alternative: ≥70% biodegradation in 28 days
```

### 8.3 Sustainability Metrics

#### 8.3.1 Renewable Carbon Index (RCI)

```
RCI = (C_bio / C_total) × 100%
```
Where:
- `C_bio` = Bio-based carbon content
- `C_total` = Total carbon content

**Target RCI**:
```
Standard grade: ≥25%
Premium grade: ≥50%
Ultimate grade: ≥75%
```

#### 8.3.2 Circular Economy Score

```
CE_score = 0.3×Renewable + 0.3×Biodegradable + 0.2×Recyclable + 0.2×LowToxicity
```

**Scoring**:
```
0-25: Conventional
26-50: Transitional
51-75: Sustainable
76-100: Highly sustainable
```

---

## 9. Data Formats

### 9.1 Fluid Specification Data

#### 9.1.1 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Universal Fluid Specification",
  "type": "object",
  "required": ["fluid_id", "name", "type", "properties"],
  "properties": {
    "fluid_id": {
      "type": "string",
      "pattern": "^UFAM-[A-Z0-9]{6}$"
    },
    "name": {
      "type": "string"
    },
    "type": {
      "enum": ["mineral", "synthetic", "bio-based", "hybrid"]
    },
    "grade": {
      "enum": ["A", "B", "C"]
    },
    "properties": {
      "type": "object",
      "properties": {
        "viscosity_40C": {
          "type": "number",
          "minimum": 40,
          "maximum": 55,
          "unit": "cSt"
        },
        "viscosity_100C": {
          "type": "number",
          "minimum": 7.5,
          "maximum": 10,
          "unit": "cSt"
        },
        "viscosity_index": {
          "type": "number",
          "minimum": 140
        },
        "pour_point": {
          "type": "number",
          "maximum": -25,
          "unit": "celsius"
        },
        "flash_point": {
          "type": "number",
          "minimum": 200,
          "unit": "celsius"
        },
        "density_15C": {
          "type": "number",
          "minimum": 850,
          "maximum": 900,
          "unit": "kg_per_m3"
        }
      }
    },
    "environmental": {
      "type": "object",
      "properties": {
        "biodegradability": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "unit": "percent"
        },
        "bio_based_content": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "unit": "percent"
        },
        "toxicity_lc50": {
          "type": "number",
          "minimum": 100,
          "unit": "mg_per_L"
        }
      }
    }
  }
}
```

### 9.2 Monitoring Data Format

#### 9.2.1 Real-Time Measurement

```json
{
  "timestamp": "2025-12-26T10:30:00.000Z",
  "sensor_id": "UFAM-SENSOR-001",
  "vehicle_id": "VEH-12345",
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194
  },
  "measurements": {
    "viscosity": {
      "value": 45.2,
      "unit": "cSt",
      "temperature": 40,
      "quality": "good"
    },
    "temperature": {
      "value": 85,
      "unit": "celsius",
      "quality": "good"
    },
    "tan": {
      "value": 1.8,
      "unit": "mg_KOH_per_g",
      "method": "potentiometric",
      "quality": "fair"
    },
    "water_content": {
      "value": 0.05,
      "unit": "percent",
      "method": "capacitance",
      "quality": "good"
    },
    "particle_count": {
      "value": 15,
      "unit": "particles_per_mL",
      "size_threshold_um": 25,
      "iso_code": "16/14/11",
      "quality": "good"
    }
  },
  "health_score": 78,
  "alerts": []
}
```

### 9.3 Replacement Recommendation Format

```json
{
  "timestamp": "2025-12-26T10:30:00.000Z",
  "vehicle_id": "VEH-12345",
  "fluid_id": "UFAM-ABC123",
  "current_condition": {
    "health_score": 45,
    "operating_hours": 5500,
    "distance_km": 85000
  },
  "recommendation": {
    "action": "plan_replacement",
    "priority": "medium",
    "timeline_days": 30,
    "reason": "TAN approaching condemning limit",
    "estimated_cost": 250.00,
    "currency": "USD"
  },
  "alternatives": [
    {
      "action": "chemical_replenishment",
      "cost": 80.00,
      "life_extension_hours": 1000
    },
    {
      "action": "filtration_enhancement",
      "cost": 120.00,
      "life_extension_hours": 1500
    }
  ]
}
```

---

## 10. API Interface

### 10.1 RESTful API Endpoints

#### 10.1.1 Fluid Analysis

**POST /api/v1/analyze**

Request:
```json
{
  "fluid_id": "UFAM-ABC123",
  "measurements": {
    "viscosity": 45.2,
    "tan": 1.8,
    "water_content": 0.05,
    "particle_count": 15,
    "temperature": 85
  },
  "operating_hours": 5000
}
```

Response:
```json
{
  "status": "success",
  "health_score": 78,
  "condition": "good",
  "remaining_life_hours": 3500,
  "recommendation": "continue_monitoring",
  "next_test_hours": 500,
  "alerts": [],
  "details": {
    "viscosity_status": "normal",
    "oxidation_status": "acceptable",
    "contamination_status": "normal"
  }
}
```

#### 10.1.2 Compatibility Check

**POST /api/v1/compatibility**

Request:
```json
{
  "fluid_a": "UFAM-ABC123",
  "fluid_b": "MINERAL-OIL-VG46",
  "mixture_ratio": 0.5
}
```

Response:
```json
{
  "status": "success",
  "compatible": true,
  "compatibility_score": 92,
  "warnings": [
    "Slight viscosity increase expected"
  ],
  "recommendations": [
    "Test mixture before full implementation",
    "Monitor viscosity after mixing"
  ]
}
```

#### 10.1.3 Replacement Schedule

**POST /api/v1/schedule**

Request:
```json
{
  "vehicle_id": "VEH-12345",
  "fluid_type": "universal-synthetic",
  "current_hours": 5000,
  "operating_conditions": "normal",
  "current_health_score": 78
}
```

Response:
```json
{
  "status": "success",
  "current_schedule": {
    "next_replacement_hours": 8500,
    "next_replacement_date": "2026-06-15",
    "confidence": 0.85
  },
  "cost_estimate": {
    "fluid_cost": 180.00,
    "labor_cost": 70.00,
    "total": 250.00,
    "currency": "USD"
  }
}
```

### 10.2 WebSocket Interface

#### 10.2.1 Real-Time Monitoring

```javascript
// Connect to WebSocket
ws://api.wia.org/v1/monitor?sensor_id=UFAM-001

// Message format
{
  "type": "measurement",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "data": {
    "viscosity": 45.2,
    "temperature": 85,
    "health_score": 78
  }
}

// Alert format
{
  "type": "alert",
  "severity": "warning",
  "message": "Water content exceeds action limit",
  "value": 0.12,
  "threshold": 0.10
}
```

### 10.3 SDK Functions

#### 10.3.1 Core Functions

```typescript
// Analyze fluid condition
function analyzeFluidCondition(
  measurements: FluidMeasurements
): FluidConditionResult;

// Calculate replacement schedule
function calculateReplacementSchedule(
  params: ScheduleParameters
): ReplacementSchedule;

// Check compatibility
function checkCompatibility(
  fluidA: string,
  fluidB: string,
  ratio: number
): CompatibilityResult;

// Monitor fluid health
function monitorFluidHealth(
  sensorId: string,
  callback: (data: FluidData) => void
): void;

// Predict remaining life
function predictRemainingLife(
  currentCondition: FluidCondition,
  operatingProfile: OperatingProfile
): RemainingLifeEstimate;
```

---

## 11. Testing Standards

### 11.1 Physical Property Tests

| Property | Test Method | Frequency | Acceptance Criteria |
|----------|-------------|-----------|---------------------|
| Kinematic Viscosity | ASTM D445 | New fluid, every change | 40-55 cSt @ 40°C |
| Viscosity Index | ASTM D2270 | New fluid | ≥140 |
| Pour Point | ASTM D97 | New fluid | ≤-25°C |
| Flash Point | ASTM D92 | New fluid | ≥200°C |
| Density | ASTM D4052 | New fluid | 850-900 kg/m³ @ 15°C |

### 11.2 Chemical Property Tests

| Property | Test Method | Frequency | Acceptance Criteria |
|----------|-------------|-----------|---------------------|
| Total Acid Number | ASTM D974 | Monthly | <4.0 mg KOH/g |
| Total Base Number | ASTM D2896 | Monthly | >50% of new fluid |
| Oxidation | FTIR | Monthly | <20 abs/cm |
| Nitration | FTIR | Monthly | <25 abs/cm |
| Water Content | ASTM D6304 | Weekly | <0.2% |

### 11.3 Performance Tests

| Test | Method | Frequency | Acceptance Criteria |
|------|--------|-----------|---------------------|
| Four-Ball Wear | ASTM D4172 | New formulation | ≤0.45 mm scar |
| Four-Ball EP | ASTM D2783 | New formulation | Weld point ≥250 kg |
| Rust Prevention | ASTM D665 | New formulation | Pass (no rust) |
| Copper Corrosion | ASTM D130 | New formulation | Maximum 1b |
| Foam Test | ASTM D892 | New formulation | ≤50 mL, 0 mL after settling |
| Air Release | ASTM D3427 | New formulation | ≤10 min @ 50°C |

### 11.4 Environmental Tests

| Test | Method | Frequency | Acceptance Criteria |
|------|--------|-----------|---------------------|
| Biodegradability | OECD 301B | New formulation | ≥60% in 28 days |
| Aquatic Toxicity | OECD 203 | New formulation | LC50 >100 mg/L |
| Bio-based Content | ASTM D6866 | Per batch | As specified |
| Ecotoxicity | OECD 202 | New formulation | EC50 >100 mg/L |

---

## 12. References

### 12.1 Standards and Specifications

1. **ASTM D445** - Standard Test Method for Kinematic Viscosity
2. **ASTM D2270** - Standard Practice for Calculating Viscosity Index
3. **ASTM D97** - Standard Test Method for Pour Point
4. **ASTM D92** - Standard Test Method for Flash Point
5. **ASTM D974** - Standard Test Method for Acid and Base Number
6. **ASTM D4172** - Standard Test Method for Wear Preventive Characteristics
7. **ISO 4406** - Hydraulic Fluid Power - Fluids - Method for Coding Level of Contamination
8. **OECD 301B** - Ready Biodegradability: CO₂ Evolution Test
9. **SAE J300** - Engine Oil Viscosity Classification
10. **ISO 6743** - Lubricants, Industrial Oils and Related Products - Classification

### 12.2 Technical Literature

1. Stachowiak, G.W., Batchelor, A.W. (2013). "Engineering Tribology", 4th Edition
2. Lansdown, A.R. (2004). "Lubrication and Lubricant Selection: A Practical Guide"
3. Mortier, R.M., Fox, M.F., Orszulik, S.T. (2010). "Chemistry and Technology of Lubricants", 3rd Edition
4. Rudnick, L.R. (2009). "Lubricant Additives: Chemistry and Applications", 2nd Edition
5. Mang, T., Dresel, W. (2007). "Lubricants and Lubrication", 2nd Edition

### 12.3 Biodegradability and Sustainability

1. **EPA Green Chemistry Program** - Safer Chemical Ingredients List
2. **EU Ecolabel** - Criteria for Lubricants (Decision 2018/1702/EU)
3. **Blue Angel** - German Ecolabel for Biodegradable Lubricants (RAL-UZ 178)
4. **ISO 14040** - Environmental Management - Life Cycle Assessment
5. **ISO 14044** - Environmental Management - Life Cycle Assessment Requirements

### 12.4 WIA Standards

- **WIA-INTENT**: Intent-based automotive control interfaces
- **WIA-OMNI-API**: Universal API gateway for vehicle systems
- **WIA-SOCIAL**: Fleet and vehicle social coordination
- **WIA-GREEN**: Environmental sustainability standards
- **WIA-AUTO-001 to WIA-AUTO-026**: Related automotive standards

---

## Appendix A: Example Calculations

### A.1 Viscosity Index Calculation

```
Given:
- Viscosity at 40°C: 46.5 cSt
- Viscosity at 100°C: 8.8 cSt

Using ASTM D2270:
1. Find L and H values from tables
2. Calculate VI = [(L - U) / (L - H)] × 100
3. Result: VI = 162
```

### A.2 Heat Transfer Calculation

```
Given:
- Flow rate: 0.5 L/min = 8.33×10⁻⁶ m³/s
- Density: 870 kg/m³
- Specific heat: 2.0 kJ/kg·K
- Temperature rise: 20°C

Mass flow rate:
ṁ = ρ × Q = 870 × 8.33×10⁻⁶ = 7.25×10⁻³ kg/s

Heat transfer:
Q = ṁ × cp × ΔT
Q = 7.25×10⁻³ × 2000 × 20
Q = 290 W
```

### A.3 Remaining Life Estimation

```
Given:
- Current TAN: 2.2 mg KOH/g
- Initial TAN: 0.8 mg KOH/g
- Operating hours: 3000 h
- Condemning TAN: 4.0 mg KOH/g

Rate of TAN increase:
dTAN/dt = (2.2 - 0.8) / 3000 = 0.000467 mg KOH/g per hour

Remaining life:
RUL = (4.0 - 2.2) / 0.000467 = 3854 hours
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-027 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
