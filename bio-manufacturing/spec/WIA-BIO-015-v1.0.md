# WIA-BIO-015: Bio-Manufacturing Specification v1.0

> **Standard ID:** WIA-BIO-015
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Bio-Manufacturing Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Upstream Processing](#2-upstream-processing)
3. [Bioreactor Systems](#3-bioreactor-systems)
4. [Cell Culture Optimization](#4-cell-culture-optimization)
5. [Downstream Processing](#5-downstream-processing)
6. [Process Analytical Technology](#6-process-analytical-technology)
7. [Quality by Design](#7-quality-by-design)
8. [GMP Compliance](#8-gmp-compliance)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for bio-manufacturing processes, covering all stages from cell line development to final product purification and quality assurance.

### 1.2 Scope

The standard covers:
- Cell line development and optimization
- Fermentation and cell culture processes
- Bioreactor design and operation
- Downstream processing and purification
- Quality control and validation
- Process analytical technology (PAT)

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to enable the production of life-saving biologics that benefit humanity while ensuring safety, quality, and sustainability.

### 1.4 Terminology

- **Titer**: Product concentration at harvest (g/L)
- **Yield**: Total product recovered relative to substrate consumed
- **Productivity**: Product generated per unit time and volume (g/L/h)
- **Viability**: Percentage of living cells in culture
- **Specific Growth Rate (μ)**: Rate of cell population increase (h⁻¹)
- **Doubling Time**: Time required for cell population to double

---

## 2. Upstream Processing

### 2.1 Cell Line Development

#### 2.1.1 Expression Systems

Common expression systems:

| System | Advantages | Product Types | Typical Yield |
|--------|-----------|---------------|---------------|
| CHO Cells | Mammalian glycosylation | Antibodies, proteins | 1-10 g/L |
| E. coli | Fast growth, high yield | Simple proteins | 5-50 g/L |
| Yeast (Pichia) | Eukaryotic folding | Enzymes, hormones | 1-15 g/L |
| Insect Cells | Complex proteins | Viral vectors | 0.1-5 g/L |
| HEK-293 | Viral production | Gene therapy vectors | 10⁶-10⁹ vp/mL |

#### 2.1.2 Clone Selection

Clone selection criteria:
```
Score = (Titer × Growth_rate × Stability) / Variability

Where:
- Titer: Product concentration (g/L)
- Growth_rate: Specific growth rate (h⁻¹)
- Stability: Passage stability (0-1)
- Variability: Batch-to-batch variation (CV%)
```

Best clones: Score > 10 with stability > 0.95

### 2.2 Media Optimization

#### 2.2.1 Basal Media Components

Essential components:
- **Carbon source**: Glucose (5-25 g/L)
- **Nitrogen source**: Amino acids, peptones
- **Vitamins**: B-complex vitamins
- **Trace elements**: Fe, Zn, Cu, Mn
- **Growth factors**: Insulin, transferrin

#### 2.2.2 Feed Strategy

Fed-batch feeding rate:
```
F(t) = (μ_set × X(t) × V(t)) / (Y_X/S × S_feed)

Where:
- F(t): Feed rate at time t (L/h)
- μ_set: Desired specific growth rate (h⁻¹)
- X(t): Cell density at time t (cells/mL or g/L)
- V(t): Culture volume at time t (L)
- Y_X/S: Yield coefficient (cells/g substrate)
- S_feed: Feed substrate concentration (g/L)
```

### 2.3 Inoculum Preparation

#### 2.3.1 Seed Train Expansion

Typical scale-up progression:
```
T-Flask (25 cm²) → Shake Flask (125 mL) →
Spinner Flask (1 L) → Wave Bag (10 L) →
Seed Bioreactor (100 L) → Production Reactor (1,000-10,000 L)
```

Scale-up ratio: 1:5 to 1:10 per step

#### 2.3.2 Inoculation Density

Optimal inoculation density:
```
X_0 = 2 × 10⁵ to 5 × 10⁵ cells/mL (mammalian)
X_0 = 1 × 10⁷ to 1 × 10⁸ cells/mL (microbial)
```

---

## 3. Bioreactor Systems

### 3.1 Reactor Types

#### 3.1.1 Stirred Tank Reactor (STR)

Volume equation:
```
V_working = 0.6 to 0.8 × V_total

Typical working volumes:
- Lab scale: 1-10 L
- Pilot scale: 100-500 L
- Production scale: 1,000-25,000 L
```

Power input:
```
P/V = K × N³ × D⁵ / V

Where:
- P/V: Power per volume (W/L)
- K: Power number (dimensionless, ~5 for turbines)
- N: Impeller speed (rps)
- D: Impeller diameter (m)
- V: Volume (L)
```

#### 3.1.2 Wave Bioreactor

Rocking motion parameters:
```
Angle: 5-12°
Rate: 10-40 rocks/min
Volume: 2-500 L (single use)
```

Advantages:
- Disposable, no cleaning validation
- Gentle mixing for shear-sensitive cells
- Rapid setup and turnaround

#### 3.1.3 Perfusion Bioreactor

Perfusion rate:
```
D = F / V

Where:
- D: Dilution rate (day⁻¹)
- F: Perfusion rate (L/day)
- V: Working volume (L)

Typical D: 0.5-2.0 day⁻¹
```

Cell retention efficiency:
```
η = (X_reactor - X_harvest) / X_reactor × 100%

Target: η > 99%
```

### 3.2 Operating Modes

#### 3.2.1 Batch Culture

Growth phases:
1. **Lag phase**: 0-12 hours (adaptation)
2. **Exponential phase**: 12-72 hours (rapid growth)
3. **Stationary phase**: 72-120 hours (production)
4. **Death phase**: >120 hours (decline)

Harvest criteria:
- Viability drops below 70%
- Or maximum titer reached
- Or day 10-14 (mammalian)

#### 3.2.2 Fed-Batch Culture

Feeding strategies:

**Exponential feeding**:
```
F(t) = F₀ × e^(μ_set × t)

Where:
- F₀: Initial feed rate (L/h)
- μ_set: Target specific growth rate (h⁻¹)
- t: Time (h)
```

**Constant rate feeding**:
```
F = constant (L/h)
```

**Bolus feeding**:
```
V_bolus added at intervals (daily or every 2 days)
```

#### 3.2.3 Continuous Culture

Steady-state conditions:
```
dX/dt = 0
dP/dt = 0
dS/dt = 0

At steady state:
X = X_ss (constant)
μ = D (specific growth rate = dilution rate)
```

Productivity at steady state:
```
Q_p = D × P_ss

Where:
- P_ss: Steady-state product concentration (g/L)
- D: Dilution rate (h⁻¹)
```

### 3.3 Critical Process Parameters (CPPs)

#### 3.3.1 pH Control

Optimal pH ranges:
- Mammalian cells: 6.9-7.4
- E. coli: 6.5-7.5
- Yeast: 5.0-6.5

pH control:
```
pH = -log[H⁺]

Control via:
- CO₂ sparging (decrease pH)
- NaOH or Na₂CO₃ addition (increase pH)
- NH₄OH addition (increase pH + nitrogen)
```

#### 3.3.2 Temperature

Optimal temperatures:
- Mammalian cells: 36-37°C
- E. coli: 30-37°C
- Yeast: 28-30°C

Temperature shift strategy:
```
T_growth = 37°C (growth phase)
T_production = 30-33°C (production phase)

Improves protein folding and reduces metabolism
```

#### 3.3.3 Dissolved Oxygen (DO)

Target DO levels:
```
DO = 30-50% of air saturation (mammalian)
DO = 20-40% of air saturation (microbial)

Control via:
- Agitation speed
- Air/O₂ flow rate
- Back pressure
```

Oxygen transfer rate (OTR):
```
OTR = k_L × a × (C* - C_L)

Where:
- k_L × a: Volumetric mass transfer coefficient (h⁻¹)
- C*: Saturation oxygen concentration (mg/L)
- C_L: Dissolved oxygen concentration (mg/L)
```

---

## 4. Cell Culture Optimization

### 4.1 Growth Kinetics

#### 4.1.1 Specific Growth Rate

Exponential growth equation:
```
X(t) = X₀ × e^(μ × t)

Where:
- X(t): Cell density at time t (cells/mL)
- X₀: Initial cell density (cells/mL)
- μ: Specific growth rate (h⁻¹)
- t: Time (h)
```

Calculating μ from data:
```
μ = ln(X₂/X₁) / (t₂ - t₁)

Or from slope of ln(X) vs time plot
```

Typical μ values:
- Mammalian cells: 0.02-0.04 h⁻¹
- E. coli: 0.4-1.0 h⁻¹
- Yeast: 0.15-0.35 h⁻¹

#### 4.1.2 Doubling Time

```
t_d = ln(2) / μ = 0.693 / μ

Where:
- t_d: Doubling time (h)
- μ: Specific growth rate (h⁻¹)
```

Typical doubling times:
- CHO cells: 18-24 hours
- E. coli: 20-40 minutes
- Yeast: 2-4 hours

### 4.2 Nutrient Consumption

#### 4.2.1 Glucose Consumption

Monod equation:
```
μ = μ_max × S / (K_s + S)

Where:
- μ_max: Maximum specific growth rate (h⁻¹)
- S: Substrate concentration (g/L)
- K_s: Saturation constant (g/L)

Typical K_s:
- Glucose: 0.01-0.1 g/L (mammalian)
- Glucose: 0.001-0.01 g/L (microbial)
```

Specific glucose consumption rate:
```
q_s = μ / Y_X/S + m_s

Where:
- q_s: Specific consumption rate (g/cell/h)
- Y_X/S: Yield coefficient (cells/g)
- m_s: Maintenance coefficient (g/cell/h)
```

#### 4.2.2 Lactate Metabolism

Lactate production:
```
q_lac = Y_lac/glc × q_glc

Where:
- q_lac: Specific lactate production (mmol/cell/h)
- Y_lac/glc: Lactate yield from glucose (mol/mol)
- q_glc: Specific glucose consumption (mmol/cell/h)

Target: Y_lac/glc < 1.5 to minimize lactate
```

Lactate inhibition threshold: >40 mM

### 4.3 Product Formation

#### 4.3.1 Specific Productivity

```
q_p = (1/X) × (dP/dt)

Where:
- q_p: Specific productivity (pg/cell/day)
- X: Cell density (cells/mL)
- dP/dt: Product formation rate (g/L/h)

Typical q_p for mAb: 20-50 pg/cell/day
```

#### 4.3.2 Product-to-Cell Yield

```
Y_P/X = P_final / ∫X dt

Where:
- Y_P/X: Product yield per cell-time (g/cell/h)
- P_final: Final product concentration (g/L)
- ∫X dt: Integral of viable cell density (IVC)
```

---

## 5. Downstream Processing

### 5.1 Harvest and Clarification

#### 5.1.1 Centrifugation

Settling velocity (Stokes' Law):
```
v = (2 × r² × (ρ_p - ρ_f) × g) / (9 × η)

Where:
- v: Settling velocity (m/s)
- r: Particle radius (m)
- ρ_p: Particle density (kg/m³)
- ρ_f: Fluid density (kg/m³)
- g: Gravitational acceleration (9.81 m/s²)
- η: Fluid viscosity (Pa·s)
```

Centrifugal force:
```
RCF = 1.118 × 10⁻⁵ × r × N²

Where:
- RCF: Relative centrifugal force (×g)
- r: Radius (cm)
- N: Rotation speed (rpm)
```

#### 5.1.2 Depth Filtration

Filter sizing:
```
V_capacity = A × Flux × t

Where:
- V_capacity: Volume capacity (L)
- A: Filter area (m²)
- Flux: Filtrate flux (L/m²/h)
- t: Filtration time (h)

Typical flux: 50-200 L/m²/h
```

### 5.2 Chromatography

#### 5.2.1 Protein A Affinity Chromatography

Binding capacity:
```
Q = Q_max × C / (K_d + C)

Where:
- Q: Bound protein (mg/mL resin)
- Q_max: Maximum binding capacity (mg/mL)
- C: Protein concentration (mg/mL)
- K_d: Dissociation constant (mg/mL)

Typical Q_max for Protein A: 30-50 g mAb/L resin
```

Dynamic binding capacity (DBC):
```
DBC = (C_load × V_breakthrough) / V_resin

Where:
- C_load: Load concentration (g/L)
- V_breakthrough: Volume at 10% breakthrough (L)
- V_resin: Resin volume (L)

Target: DBC > 30 g/L at 6 min residence time
```

#### 5.2.2 Ion Exchange Chromatography

Resin capacity:
```
Q = n × F × e^(-ΔG/RT)

Where:
- Q: Bound protein (mg/mL)
- n: Number of binding sites
- F: Fraction of available sites
- ΔG: Gibbs free energy of binding (kJ/mol)
- R: Gas constant (8.314 J/mol·K)
- T: Temperature (K)
```

Conductivity requirements:
- CEX (cation exchange): 3-8 mS/cm
- AEX (anion exchange): 5-15 mS/cm

#### 5.2.3 Hydrophobic Interaction Chromatography (HIC)

Salt concentration:
```
Binding: 1.0-1.5 M (NH₄)₂SO₄ or 0.5-1.0 M NaCl
Elution: Decreasing salt gradient to 0 M
```

### 5.3 Viral Inactivation

#### 5.3.1 Low pH Hold

Conditions:
```
pH: 3.3-3.8
Time: 30-60 minutes
Temperature: Room temperature (20-25°C)

Viral reduction: >4 log₁₀
```

#### 5.3.2 Solvent/Detergent Treatment

Typical formulation:
```
0.3% Tri-n-butyl phosphate (TNBP)
1.0% Triton X-100 or Tween 80
Time: 1-4 hours
Temperature: Room temperature

Viral reduction: >4 log₁₀ for enveloped viruses
```

### 5.4 Concentration and Buffer Exchange

#### 5.4.1 Ultrafiltration/Diafiltration (UF/DF)

Permeate flux:
```
J = ΔP / (μ × R_m)

Where:
- J: Permeate flux (L/m²/h)
- ΔP: Transmembrane pressure (bar)
- μ: Viscosity (Pa·s)
- R_m: Membrane resistance (m⁻¹)

Typical flux: 30-100 L/m²/h (MWCO 30 kDa)
```

Concentration factor:
```
CF = V_initial / V_final

Where:
- CF: Concentration factor
- V_initial: Starting volume (L)
- V_final: Final volume (L)

Typical CF: 5-10×
```

Diafiltration volumes:
```
DV = 5-10 × V_final

Where:
- DV: Diafiltration volume (L)
- V_final: Final retentate volume (L)

Removes >99% of original buffer
```

---

## 6. Process Analytical Technology (PAT)

### 6.1 Real-Time Monitoring

#### 6.1.1 Online Sensors

Critical measurements:
- **pH**: ±0.05 accuracy
- **Temperature**: ±0.1°C accuracy
- **Dissolved Oxygen**: ±2% accuracy
- **Pressure**: ±0.01 bar accuracy
- **Agitation**: ±5 rpm accuracy

#### 6.1.2 At-Line Analysis

Rapid measurements (5-15 min):
- Glucose: 0.1-25 g/L range
- Lactate: 0.1-40 mM range
- Glutamine: 0.1-10 mM range
- Ammonia: 0.1-10 mM range
- Viable cell density: 10⁵-10⁷ cells/mL

#### 6.1.3 Spectroscopic Methods

**Raman Spectroscopy**:
```
Measures:
- Glucose concentration
- Product titer
- Cell density
- Metabolite levels

Sampling: Non-invasive, through reactor wall
Frequency: Every 15-30 minutes
```

**Near-Infrared (NIR)**:
```
Measures:
- Total protein
- Cell concentration
- Medium components

Calibration: Multivariate models (PLS, PCR)
```

### 6.2 Process Control

#### 6.2.1 PID Control

PID equation:
```
u(t) = K_p × e(t) + K_i × ∫e(t)dt + K_d × de(t)/dt

Where:
- u(t): Control output
- K_p: Proportional gain
- K_i: Integral gain
- K_d: Derivative gain
- e(t): Error signal (setpoint - measured)

Used for: pH, DO, temperature, level control
```

#### 6.2.2 Advanced Process Control

Model Predictive Control (MPC):
```
Minimize: J = Σ(y_pred - y_target)² + λ × Σ(Δu)²

Where:
- J: Cost function
- y_pred: Predicted output
- y_target: Target setpoint
- Δu: Control move
- λ: Move suppression factor

Applications:
- Feed rate optimization
- Multi-variable control
- Constraint handling
```

---

## 7. Quality by Design (QbD)

### 7.1 Design Space

#### 7.1.1 Critical Quality Attributes (CQAs)

For monoclonal antibodies:
- **Purity**: >95% (HPLC)
- **Aggregates**: <5% (SEC-HPLC)
- **Charge variants**: Acidic + Basic < 40%
- **Glycosylation**: G0F 35-55%, High mannose <5%
- **Host cell proteins**: <100 ppm
- **DNA**: <10 ng/dose
- **Endotoxin**: <5 EU/mg

#### 7.1.2 Design of Experiments (DOE)

Full factorial design:
```
N = k^n

Where:
- N: Number of experiments
- k: Number of levels per factor
- n: Number of factors

Example: 3 factors, 3 levels = 3³ = 27 runs
```

Fractional factorial:
```
Resolution III: Main effects
Resolution IV: Main effects + some 2-way interactions
Resolution V: Main effects + all 2-way interactions
```

Response surface methodology:
```
Y = β₀ + Σβᵢ × Xᵢ + Σβᵢᵢ × Xᵢ² + Σβᵢⱼ × Xᵢ × Xⱼ

Where:
- Y: Response (CQA)
- Xᵢ: Factor i (CPP)
- βᵢ: Linear coefficient
- βᵢᵢ: Quadratic coefficient
- βᵢⱼ: Interaction coefficient
```

### 7.2 Process Capability

#### 7.2.1 Capability Indices

Process capability:
```
C_p = (USL - LSL) / (6 × σ)

Where:
- USL: Upper specification limit
- LSL: Lower specification limit
- σ: Process standard deviation

Target: C_p > 1.33 (capable process)
```

Process performance:
```
C_pk = min[(USL - μ)/(3σ), (μ - LSL)/(3σ)]

Where:
- μ: Process mean

Target: C_pk > 1.33
```

---

## 8. GMP Compliance

### 8.1 Documentation

#### 8.1.1 Batch Records

Required documentation:
- Master batch record (MBR)
- Batch production record (BPR)
- Deviation reports
- Change controls
- Environmental monitoring
- Equipment logs

#### 8.1.2 Validation

Process validation stages:
1. **Stage 1**: Process design (QbD)
2. **Stage 2**: Process qualification
   - Installation Qualification (IQ)
   - Operational Qualification (OQ)
   - Performance Qualification (PQ)
3. **Stage 3**: Continued process verification

Validation requirements:
```
Minimum 3 consecutive successful batches
All CQAs within specifications
Process capability demonstrated (C_pk > 1.33)
```

### 8.2 Cleaning Validation

#### 8.2.1 Acceptance Criteria

Product residue:
```
Limit = (MACO × Batch_size_next) / (SF × Shared_equipment_surface)

Where:
- MACO: Maximum allowable carryover (ppm)
- SF: Safety factor (typically 1000)

Or: <10 ppm of previous product
Or: <0.1% of therapeutic dose
```

Cleaning agent residue:
```
Limit: <10 ppm of cleaning agent
Or: Below toxicological threshold
```

#### 8.2.2 Sampling

Swab sampling:
```
Area = 25 cm² (typical swab area)
Recovery efficiency: >50%

Contamination = (Amount_detected / Recovery) × (Total_area / Swab_area)
```

---

## 9. Implementation Guidelines

### 9.1 Required Components

Any WIA-BIO-015 compliant system must include:

1. **Yield Calculator**: Compute product yield and productivity
2. **Bioreactor Monitor**: Track critical process parameters
3. **Optimization Engine**: Design of experiments and modeling
4. **Purification Designer**: Chromatography sequence optimization
5. **Quality Control**: Track CQAs and process capability

### 9.2 API Interface

#### 9.2.1 Calculate Yield
```typescript
interface YieldRequest {
  productConcentration: number;  // g/L
  finalVolume: number;           // L
  substrateConcentration: number;// g/L
  initialVolume: number;         // L
}

interface YieldResponse {
  yield: number;                 // dimensionless
  totalProduct: number;          // g
  totalSubstrate: number;        // g
  efficiency: 'excellent' | 'good' | 'fair' | 'poor';
}
```

#### 9.2.2 Monitor Bioreactor
```typescript
interface BioreactorStatus {
  reactorId: string;
  cellDensity: number;           // cells/mL
  viability: number;             // %
  pH: number;
  temperature: number;           // °C
  dissolvedOxygen: number;       // %
  productTiter?: number;         // g/L
}

interface MonitoringResult {
  status: 'optimal' | 'acceptable' | 'warning' | 'critical';
  warnings: string[];
  recommendations: string[];
  trends: ParameterTrend[];
}
```

#### 9.2.3 Design Purification
```typescript
interface PurificationConfig {
  productType: 'antibody' | 'protein' | 'enzyme' | 'vaccine';
  scale: number;                 // L
  targetPurity: number;          // %
  targetRecovery: number;        // %
}

interface PurificationProtocol {
  steps: ChromatographyStep[];
  estimatedRecovery: number;     // %
  estimatedPurity: number;       // %
  estimatedCost: number;         // $ per g
  duration: number;              // hours
}
```

### 9.3 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| B001 | Low cell viability | Check culture conditions |
| B002 | High lactate | Reduce glucose feed |
| B003 | Low titer | Optimize expression |
| B004 | pH excursion | Calibrate sensors |
| B005 | Contamination detected | Investigate, discard batch |
| B006 | Equipment failure | Maintenance required |

---

## 10. References

### 10.1 Scientific Literature

1. Birch, J.R., Racher, A.J. (2006). "Antibody production"
2. Mandenius, C.F., Brundin, A. (2008). "Bioprocess optimization using DOE"
3. FDA Guidance (2011). "Process Validation: General Principles"
4. ICH Q8(R2) (2009). "Pharmaceutical Development"
5. Kelley, B. (2009). "Industrialization of mAb production"

### 10.2 Bio-Manufacturing Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Avogadro's number | N_A | 6.022 × 10²³ mol⁻¹ |
| Gas constant | R | 8.314 J/mol·K |
| Standard temperature | T | 298 K (25°C) |
| Standard pressure | P | 1 atm (101.325 kPa) |

### 10.3 WIA Standards

- WIA-BIO-002: Genome Sequencing
- WIA-BIO-005: Drug Discovery
- WIA-BIO-008: Tissue Engineering
- WIA-DATA: Data management
- WIA-INTENT: Intent-based interfaces

---

## Appendix A: Example Calculations

### A.1 Product Yield Calculation

```
Given:
- Product concentration: 5.2 g/L
- Final volume: 1000 L
- Substrate concentration: 50 g/L
- Initial volume: 1000 L

Calculation:
- Total product: 5.2 × 1000 = 5,200 g
- Total substrate: 50 × 1000 = 50,000 g
- Yield: 5,200 / 50,000 = 0.104 (10.4%)

Interpretation:
- 10.4% conversion efficiency
- Typical for mammalian cell culture
- Can be improved with fed-batch feeding
```

### A.2 Volumetric Productivity

```
Given:
- Final concentration: 5.2 g/L
- Initial concentration: 0 g/L
- Culture time: 240 hours (10 days)

Calculation:
- Q_p = (5.2 - 0) / 240
- Q_p = 0.0217 g/L/h
- Q_p = 0.52 g/L/day

Comparison:
- Typical mAb productivity: 0.3-0.8 g/L/day
- This performance: Good
```

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-015 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
