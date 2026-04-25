# WIA-SEMI-020: Reliability Specification

## 1. Scope
Defines reliability requirements, test methods, and acceptance criteria for advanced semiconductor packaging technologies.

## 2. Qualification Test Requirements

### 2.1 Temperature Cycling Test (TCT)
**Standard**: JEDEC JESD22-A104
**Conditions**:
- Temperature range: -40°C to +125°C (standard), -55°C to +150°C (extended)
- Dwell time: 10-30 minutes at each extreme
- Transition time: <10 minutes
- Cycles: 500-3000 cycles depending on application

**Acceptance Criteria**:
- Zero failures up to specified cycle count
- Parametric shifts within ±5% of initial values
- No visual defects (cracks, delamination)

**Failure Modes**: Solder joint fatigue, die cracking, delamination, microbump failures

### 2.2 High Temperature Storage Life (HTSL)
**Standard**: JEDEC JESD22-A103
**Conditions**:
- Temperature: 150°C (storage condition, no electrical bias)
- Duration: 1000-2000 hours

**Acceptance Criteria**:
- Zero failures
- Parametric drift <5%

**Failure Modes**: Intermetallic compound growth, oxidation, material degradation

### 2.3 Temperature, Humidity, Bias (THB)
**Standard**: JEDEC JESD22-A101
**Conditions**:
- Temperature: 85°C
- Humidity: 85% RH
- Bias: Operating voltage applied
- Duration: 1000-2000 hours

**Acceptance Criteria**:
- Zero failures
- Leakage current increase <2×
- Functional parameters within limits

**Failure Modes**: Electrochemical migration, corrosion, delamination

### 2.4 Highly Accelerated Stress Test (HAST)
**Standard**: JEDEC JESD22-A110
**Conditions**:
- Biased HAST: 130°C, 85% RH, 2 atm, 96-168 hours
- Unbiased HAST: 143°C, 85% RH, 2 atm, 96 hours

**Acceptance Criteria**: Zero failures, no electrical parameter degradation

**Acceleration Factor**: ~100× compared to standard THB test

### 2.5 Preconditioning and Moisture Sensitivity Level (MSL)
**Standard**: JEDEC J-STD-020
**Procedure**:
1. Moisture preconditioning per MSL level
2. Reflow simulation: 3× reflow at peak 260°C
3. Evaluation: Acoustic microscopy (SAM), electrical test

**MSL Classifications**:
- MSL 1: Unlimited floor life (≤30°C/85% RH)
- MSL 2: 1 year (≤30°C/60% RH)
- MSL 2a: 4 weeks
- MSL 3: 168 hours
- MSL 4: 72 hours
- MSL 5: 48 hours
- MSL 5a: 24 hours
- MSL 6: Mandatory bake before use

**Acceptance**: No popcorning, no delamination >25% of interface area

### 2.6 Autoclave/Pressure Cooker Test (PCT)
**Standard**: JEDEC JESD22-A102
**Conditions**: 121°C, 100% RH, 2 atm, 96-168 hours

**Acceptance**: No seal failures, no delamination, parametric shifts <10%

### 2.7 Board-Level Reliability (BLR)
**Standard**: JEDEC JESD22-B111 (thermal cycling), JESD22-B110A (temperature cycling)
**Conditions**:
- Thermal cycling: -40°C to +125°C or 0°C to +100°C
- Dwell: 10-15 minutes
- Cycles: 500-3000 (characteristic lifetime >1500 cycles)

**Test Board**: 1.6mm FR-4 or industry-standard test board

**Monitoring**: Daisy chain resistance, failure defined as first occurrence of >300Ω or >20% increase

**Acceptance**: Characteristic lifetime (η63.2%) >1500 cycles for consumer, >2500 for automotive

### 2.8 Drop Test
**Standard**: JEDEC JESD22-B111
**Conditions**:
- Shock level: 1500 G
- Pulse duration: 0.5 ms (half-sine wave)
- Orientations: Multiple (typically 5 faces + 1 corner)
- Drops: 30 drops minimum per orientation
- Board: 1.0 or 1.6 mm FR-4

**Acceptance**: Zero solder joint failures, continuity maintained

**Applicable**: Mobile devices, handheld equipment

### 2.9 Vibration Test
**Standard**: MIL-STD-883 Method 2007, JEDEC JESD22-B103
**Conditions**:
- Frequency range: 20-2000 Hz
- Vibration profile: Random or sinusoidal
- Duration: 12 hours (4 hours per axis)
- Magnitude: Per MIL-STD-883 or automotive standards (e.g., AEC-Q100)

**Acceptance**: Zero failures, no visible damage

**Applicable**: Automotive, industrial, aerospace

### 2.10 High Temperature Operating Life (HTOL)
**Standard**: JEDEC JESD22-A108
**Conditions**:
- Temperature: 125-150°C junction temperature
- Bias: Operating voltage, dynamic operation
- Duration: 1000-2000 hours

**Acceptance Criteria**:
- Failure rate: <100 FIT (failures per billion device-hours) at use conditions
- Parametric drift: <10% of initial values

**Application**: Used to screen for early-life failures, validate operating lifetime

## 3. Sample Size and Statistical Requirements

### 3.1 Minimum Sample Sizes
**Consumer applications**: 77-231 samples per condition (for 90% confidence)
**Industrial**: 231-462 samples
**Automotive (AEC-Q100)**: ≥3000 samples across all conditions

### 3.2 Zero-Failure Qualification
For zero failures observed:
- n samples, 90% confidence: Defect rate <2.3/n (in %)
- Example: 1000 samples, 0 failures → 90% confidence of <230 PPM

### 3.3 Weibull Analysis
**Parameters**:
- β (shape): β<1 (infant mortality), β≈1 (random failures), β>1 (wear-out)
- η (scale): Characteristic lifetime (63.2% cumulative failures)

**Requirements**: Confidence bounds on Weibull parameters within specified ranges

## 4. Reliability Metrics

### 4.1 Mean Time To Failure (MTTF)
**Definition**: Expected time to failure for non-repairable devices

**Requirements**:
- Consumer: ≥5 years at use conditions
- Industrial: ≥10 years
- Automotive: ≥15 years

**Calculation**: From qualification test data using acceleration factors

### 4.2 Failure Rate
**Units**: FIT (Failures In Time) = failures per 10⁹ device-hours

**Requirements**:
- Consumer: <500 FIT
- Industrial: <200 FIT
- Automotive: <100 FIT

### 4.3 Acceleration Factors
**Arrhenius Equation**: AF = exp[(Ea/k) × (1/Tuse - 1/Ttest)]
- Ea: Activation energy (eV)
- k: Boltzmann constant (8.617×10⁻⁵ eV/K)
- T: Temperature (Kelvin)

**Typical Activation Energies**:
- Electromigration: 0.6-0.9 eV
- TDDB: 0.3-1.5 eV
- Corrosion: 0.7-0.9 eV
- Intermetallic growth: 0.5-1.0 eV

**Coffin-Manson (thermal cycling)**: Nf = A × (ΔT)⁻ⁿ
- n: Coffin-Manson exponent (typically 2-3)
- ΔT: Temperature excursion

## 5. Failure Analysis Requirements

### 5.1 Non-Destructive Analysis
**X-Ray Inspection**: Identify voids, cracks, misalignment
**Acoustic Microscopy (SAM)**: Detect delamination, voids
**Electrical Test**: Characterize failure signature

### 5.2 Destructive Analysis
**Cross-Sectioning**: Polish and examine interfaces, solder joints
**SEM/EDS**: High-resolution imaging and elemental analysis
**FIB/TEM**: Nanoscale characterization of failure mechanisms

### 5.3 Failure Mode Classification
**Systematic failures**: Design or process issues requiring corrective action
**Random failures**: Statistical occurrences within acceptable limits
**Infant mortality**: Early-life failures (burn-in can screen)

## 6. Special Requirements for Advanced Packages

### 6.1 TSV Reliability
**Tests**:
- Thermal cycling: -40 to 125°C, >1000 cycles
- HTOL: Evaluate electromigration, stress migration
- TDDB: High voltage stress testing

**Acceptance**: No TSV opens, resistance increase <20%, no transistor degradation in keep-out zones

### 6.2 Microbump Reliability
**Tests**:
- Thermal cycling: More severe than C4 bumps due to finer pitch
- Electromigration: Current density limits more critical
- Underfill: Essential for reliability

**Acceptance**: Characteristic lifetime >2000 cycles

### 6.3 HBM Reliability
**Tests**:
- Functional test at temperature: Validate performance across range
- Thermal cycling: Validate stack integrity
- Data retention: At elevated temperature

**Acceptance**: No bit errors, stack integrity maintained

### 6.4 Chiplet Reliability
**Tests**:
- Inter-chiplet communication: Validate across temperature and lifetime
- Multi-chiplet thermal cycling: Assess cumulative stress
- Known good die (KGD) testing: Pre-integration screening

**Acceptance**: All chiplets functional, communication error-free

## 7. Automotive-Specific Requirements (AEC-Q100)

### 7.1 Temperature Grades
- Grade 0: -40 to +150°C/-175°C
- Grade 1: -40 to +125°C/-150°C
- Grade 2: -40 to +105°C/-125°C
- Grade 3: -40 to +85°C

### 7.2 Qualification Tests
Includes all standard tests plus automotive-specific:
- Extended temperature cycling (-65 to +150°C)
- Biased HAST: 130°C/85%RH, 264 hours
- Power temperature cycling (PTC)

### 7.3 Sample Sizes
Total >3000 devices across all tests

### 7.4 Zero-Failure Requirement
Automotive qualification typically requires zero failures across all tests

## 8. Documentation Requirements

### 8.1 Qualification Report
Must include:
- Test plan and procedures
- Sample size and lot information
- Test equipment calibration records
- Raw data and statistical analysis
- Failure analysis reports (if any)
- Weibull plots and parameters
- Conclusions and approval

### 8.2 Ongoing Reliability Monitoring
**Production monitoring**:
- Sample testing from production lots
- Trend analysis of key parameters
- Field return analysis
- Continuous improvement programs

## 9. Compliance

Compliance demonstrated by:
- Completion of qualification test matrix
- Meeting acceptance criteria for all tests
- Zero systematic failures (random failures within acceptable limits)
- Documented qualification report approved by reliability organization
- Ongoing production monitoring and field reliability tracking

---

**© 2025 SmileStory Inc. / WIA**
弘익人間 (홍익인간) · Benefit All Humanity
