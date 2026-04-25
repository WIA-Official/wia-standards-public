#!/bin/bash

# Create thermal-spec.md
cat > WIA-SEMI-020-thermal-spec.md << 'EOF'
# WIA-SEMI-020: Thermal Management Specification

## 1. Scope
This specification defines thermal management requirements, test methods, and design guidelines for advanced semiconductor packaging.

## 2. Thermal Resistance Specifications

### 2.1 Package Thermal Resistance
- θJC: 0.1-1.0 °C/W for high-performance packages
- θJB: 5-20 °C/W depending on PCB design
- θJA: System and cooling dependent

### 2.2 Measurement Methods
Thermal resistance measured per JEDEC JESD51 series standards using thermal test dies with integrated heaters and temperature sensors.

## 3. Junction Temperature Limits

### 3.1 Maximum Ratings
- Consumer/Commercial: 125°C maximum
- Industrial: 150°C maximum
- Automotive (AEC-Q100): 
  - Grade 0: 150-175°C
  - Grade 1: 125-150°C
  - Grade 2: 105-125°C

### 3.2 Continuous Operating Limits
Typical continuous junction temperature: 105°C to ensure reliability over product lifetime.

## 4. Thermal Interface Materials (TIM) Requirements

### 4.1 TIM Properties
- Thermal conductivity: 1-80 W/m·K depending on application
- Bond line thickness: 25-500 μm
- Thermal resistance: <0.05 °C·cm²/W
- Operating temperature range: -40 to 150°C
- Pump-out resistance: <10% thickness change after 1000 thermal cycles

### 4.2 TIM Types and Applications
**Thermal Greases/Pastes**: 1-5 W/m·K, easy application, moderate performance
**Phase Change Materials**: 2-8 W/m·K, solid at room temp, flows during operation
**Thermal Pads**: 1-5 W/m·K, pre-formed, consistent thickness
**Solder TIM**: 20-80 W/m·K, excellent performance, difficult rework
**Liquid Metal TIM**: 20-70 W/m·K, highest performance, requires containment

## 5. Heat Sink Requirements

### 5.1 Heat Sink Performance
Heat sink thermal resistance: 0.1-5 °C/W depending on size, airflow, and cooling method.

### 5.2 Heat Sink Attachment
- Mounting pressure: 30-150 psi
- Attachment method: Screw-down, clip, or adhesive
- Coplanarity: ±50 μm across contact area
- Surface finish: <1.6 μm Ra

### 5.3 Heat Sink Materials
- Aluminum: Low cost, thermal conductivity ~200 W/m·K
- Copper: Higher performance, thermal conductivity ~400 W/m·K
- Vapor chamber/heat pipe: Very high effective thermal conductivity (>1000 W/m·K effective)

## 6. Package Thermal Design

### 6.1 2.5D Thermal Design
**Heat Paths**: Through silicon interposer to substrate, and top surface to heat sink/spreader.
**Thermal Vias**: Use thermal vias in substrate (diameter ≥250 μm, spacing ≤500 μm) to improve heat extraction.
**Die Placement**: Optimize die placement to avoid thermal hotspots, maintain >2mm spacing between high-power dies when possible.

### 6.2 3D Thermal Design
**Vertical Heat Flow**: Each stacked die adds thermal resistance; limit stack height based on power budget.
**TSV Thermal Enhancement**: Use dedicated thermal TSVs (>10% of total TSV area recommended).
**Die Thinning**: Thinner dies (30-50 μm) reduce thermal resistance compared to thicker dies (100 μm+).
**Thermal Management**: Consider active cooling (fans, liquid cooling) for high-power 3D stacks (>100W total).

### 6.3 Fan-Out Thermal Design
**Direct Heat Path**: Die backside exposed or with thin molding provides direct thermal path to heat sink.
**Molding Compound**: Use high thermal conductivity molding compounds (2-5 W/m·K) for improved performance.
**RDL Thermal Design**: Maximize copper density in RDL layers for heat spreading; use thermal vias where appropriate.

## 7. Thermal Simulation Requirements

### 7.1 Simulation Tools
Use finite element analysis (FEA) tools capable of multi-physics thermal simulation: ANSYS Mechanical, Siemens FloTHERM, Cadence Celsius, or equivalent.

### 7.2 Model Requirements
**Geometry**: Accurate 3D geometry including all package layers, dies, substrate, TIM, heat sink.
**Materials**: Temperature-dependent material properties where significant.
**Power Maps**: Realistic die power maps from electrical simulation or measurement.
**Boundary Conditions**: Appropriate ambient temperature, convective heat transfer coefficients, airflow.

### 7.3 Validation
Thermal simulations must be validated against physical measurements using thermal test dies or IR imaging within ±10% agreement.

## 8. Thermal Testing and Characterization

### 8.1 Thermal Test Die
Thermal test dies shall include:
- Resistive heaters covering >50% of die area
- Temperature sensors (diodes, resistors) at multiple locations
- Power and temperature sense connections

### 8.2 Junction-to-Case Thermal Resistance (θJC)
Measured per JEDEC JESD51-2:
- Mount package on cold plate at controlled temperature
- Apply known power to test die
- Measure junction temperature rise
- θJC = ΔT / P

### 8.3 Junction-to-Ambient Thermal Resistance (θJA)
Measured per JEDEC JESD51-2A in still air:
- Mount package on test board in still air chamber
- Apply known power
- Measure junction and ambient temperatures
- θJA = (TJ - TA) / P

### 8.4 Thermal Transient Testing
Measure thermal time constants and package structure using thermal transient testing per JEDEC JESD51-14.

### 8.5 Infrared Thermal Imaging
Use IR camera to measure surface temperature distributions:
- Calibrate for emissivity variations
- Compare to simulation results
- Identify hotspots and thermal gradients

## 9. Thermal Reliability

### 9.1 Temperature Cycling Effects
Thermal cycling induces stress due to CTE mismatch:
- Silicon: 2.6 ppm/°C
- Molding compound: 10-30 ppm/°C
- Substrate: 15-18 ppm/°C
- Solder: 20-25 ppm/°C

### 9.2 Electromigration
Electromigration lifetime follows Black's equation; temperature critically affects lifetime (doubles approximately every 10°C reduction).

### 9.3 Time-Dependent Dielectric Breakdown (TDDB)
TDDB follows Arrhenius temperature dependence with activation energy typically 0.3-1.5 eV.

## 10. Cooling Solutions

### 10.1 Air Cooling
**Natural Convection**: h = 5-25 W/m²·K, suitable for low power (<10W packages).
**Forced Air**: h = 25-250 W/m²·K with fan; performance depends on airflow rate and fin design.

### 10.2 Liquid Cooling
**Cold Plates**: Heat exchanger attached to package with liquid flowing through internal channels; h = 500-10,000 W/m²·K.
**Immersion Cooling**: Package submerged in dielectric fluid; excellent thermal performance for high-density systems.
**Microfluidic Cooling**: Microchannels integrated in package or die; can achieve h > 10,000 W/m²·K; enables >1000 W/cm² heat flux.

### 10.3 Advanced Cooling Technologies
**Vapor Chambers**: Spreads heat efficiently; effective thermal conductivity >1000 W/m·K
**Heat Pipes**: Transports heat over distance with minimal temperature drop
**Thermoelectric Coolers**: Peltier effect cooling; can achieve sub-ambient temperatures but power-hungry (COP typically <1)
**Phase Change Materials**: Provides thermal buffering during transient loads

## 11. Design Guidelines

### 11.1 Thermal-Aware Floorplanning
- Distribute high-power blocks across die area
- Avoid concentration of power dissipation
- Place highest power blocks nearest to best thermal path
- Maintain thermal guard bands around temperature-sensitive circuits

### 11.2 Power Management
- Implement dynamic voltage and frequency scaling (DVFS)
- Use power gating for unused blocks
- Thermal throttling as last-resort protection
- Monitor junction temperature with on-die sensors

### 11.3 Package Selection
Select package type based on thermal requirements:
- Low power (<5W): Standard packages with natural convection adequate
- Medium power (5-50W): Enhanced packages with heat spreaders or small heat sinks
- High power (50-200W): 2.5D or advanced packages with substantial heat sinks and forced air
- Very high power (>200W): Specialized cooling (liquid cooling, high-performance heat sinks)

## 12. Thermal Specifications Summary Table

| Package Type | θJC (°C/W) | Max Power (W) | Cooling Method | TIM Required |
|-------------|-----------|--------------|----------------|--------------|
| Standard BGA | 2-5 | 5-20 | Natural/Forced air | Optional |
| Enhanced BGA | 0.5-2 | 20-80 | Forced air + heatsink | Yes |
| 2.5D with HBM | 0.2-0.8 | 100-300 | Large heatsink + fan | Yes |
| Fan-out WLP | 1-3 | 10-40 | Heat spreader | Yes |
| 3D Stack | 0.5-2 | 30-150 | Advanced cooling | Yes |

## 13. Compliance and Documentation

### 13.1 Design Documentation
Required thermal design documentation:
- Thermal simulation results showing junction temperatures
- Power map used for simulation
- Material properties table
- Cooling solution specification
- Thermal test plan and results

### 13.2 Qualification
Thermal qualification shall demonstrate:
- Junction temperatures remain within specifications across operating conditions
- Thermal resistance meets specifications
- No thermal runaway or excessive temperature gradients
- Reliability requirements met across temperature range

---

**© 2025 SmileStory Inc. / WIA**
弘益人間 (홍익인간) · Benefit All Humanity
EOF

# Create electrical-spec.md  
cat > WIA-SEMI-020-electrical-spec.md << 'EOF'
# WIA-SEMI-020: Electrical Specification

## 1. Scope
Defines electrical requirements for advanced semiconductor packaging including signal integrity, power integrity, and high-speed interface specifications.

## 2. Signal Integrity Requirements

### 2.1 Impedance Control
**Single-ended signals**: 45-55 Ω ±10%
**Differential pairs**: 90-110 Ω ±10%
**Power/Ground**: <100 mΩ per connection
**Measurement**: TDR (Time Domain Reflectometry) or VNA (Vector Network Analyzer)

### 2.2 Insertion Loss
**Requirements**: <3 dB at Nyquist frequency for critical high-speed signals
**Measurement**: S21 parameters using VNA
**Contributors**: Conductor loss (skin effect, roughness), dielectric loss

### 2.3 Return Loss
**Requirements**: >10 dB at Nyquist frequency for critical signals
**Measurement**: S11 parameters using VNA
**Causes**: Impedance discontinuities at vias, bumps, plane transitions

### 2.4 Crosstalk
**Near-end crosstalk (NEXT)**: <-30 dB
**Far-end crosstalk (FEXT)**: <-30 dB
**Measurement**: S-parameters (S31, S41 for adjacent traces)
**Mitigation**: Adequate trace spacing, ground shielding, differential signaling

### 2.5 Inter-Symbol Interference (ISI)
**Requirement**: <0.25 UI (Unit Interval) at receiver
**Measurement**: Eye diagram analysis
**Mitigation**: Equalization (DFE, FFE), proper termination

## 3. Power Integrity Requirements

### 3.1 Power Distribution Network (PDN)

**DC Resistance**: <10 mΩ from power supply to die for high-current rails
**Target Impedance (Ztarget)**: Ztarget = Vripple / Itransient
- Typical values: 1-10 mΩ for high-performance processors
- Frequency range: DC to >1 GHz

**PDN Impedance Profile**:
- DC to 1 MHz: Dominated by bulk capacitance and resistance
- 1-100 MHz: Package and board capacitance region
- 100 MHz-1 GHz: On-die capacitance dominates

### 3.2 Voltage Regulation
**Static voltage drop (IR drop)**: <5% of nominal voltage
**Dynamic voltage droop**: <±5% of nominal voltage during transients
**Ripple voltage**: <50 mV peak-to-peak at operating frequency

### 3.3 Decoupling Capacitance
**On-die capacitance**: MOS capacitance, integrated deep trench caps
**Package capacitance**: Discrete capacitors on package or embedded in substrate
**Board capacitance**: Bulk capacitors, ceramic capacitors at multiple values

**Capacitor Selection**:
- 1-10 μF bulk capacitors: Low-frequency (<100 kHz)
- 0.1-1 μF ceramics: Mid-frequency (100 kHz-10 MHz)
- 10-100 nF ceramics: High-frequency (10-100 MHz)
- <10 nF ceramics: Very high-frequency (>100 MHz)

### 3.4 Power Sequencing
**Requirements**: Proper power-up and power-down sequencing to avoid latch-up and stress
**Ramp rate**: Typically 0.1-10 V/ms
**Tolerance**: Sequence timing ±10 ms

## 4. High-Speed Interface Specifications

### 4.1 SerDes (Serializer/Deserializer)
**Data rates**: 1-112 Gbps per lane
**Protocols**: PCIe 6.0, Ethernet 800G, UCIe, CXL
**Bit Error Rate (BER)**: <10⁻¹² for standard, <10⁻¹⁵ for high-reliability
**Jitter budget**: Total jitter <0.3 UI at BER=10⁻¹²

**Equalization**:
- Transmit: Pre-emphasis, de-emphasis
- Receive: CTLE (Continuous Time Linear Equalization), DFE (Decision Feedback Equalization)

### 4.2 UCIe Electrical Specifications
**Data rates**: 2-32 Gbps per pin (roadmap to higher)
**Signaling**: Single-ended or differential
**BER**: <10⁻¹⁵ target
**Latency**: Single-digit nanoseconds for PHY
**Power efficiency**: <0.5 pJ/bit target for standard package

**Link Training**: Auto-negotiation of link width, data rate, equalization settings

### 4.3 HBM Interface
**Data rate**: 2-6.4 Gbps per pin (HBM3)
**Interface width**: 1024 bits per stack (8 or 16 channels × 64 or 128 bits)
**Bandwidth**: Up to 819 GB/s per stack (HBM3)
**Voltage**: 1.1-1.2V typical
**Termination**: On-die termination (ODT)

## 5. ESD and Latch-Up Protection

### 5.1 ESD Requirements
**Human Body Model (HBM)**: ≥2 kV per JEDEC JESD22-A114
**Charged Device Model (CDM)**: ≥500 V per JEDEC JESD22-C101
**Machine Model (MM)**: ≥200 V per JEDEC JESD22-A115

**ESD Protection Structures**:
- Primary protection at pads
- Secondary protection at core interface
- Power clamps between power domains

### 5.2 Latch-Up Requirements
**Latch-up immunity**: ≥100 mA per JEDEC JESD78
**Prevention**: Guard rings, substrate contacts, ESD clamps

## 6. Interconnect Electrical Properties

### 6.1 TSV Electrical Properties
**Resistance**: <50 mΩ per TSV (5-10 μm diameter, 50-100 μm length)
**Capacitance**: 5-10 fF per TSV
**Inductance**: <10 pH per TSV
**Isolation resistance**: >100 GΩ at 125°C
**Breakdown voltage**: >100 V

### 6.2 Microbump Electrical Properties
**Resistance**: <10 mΩ per bump (40-55 μm pitch)
**Inductance**: <10 pH per bump
**Capacitance**: <5 fF per bump
**Current carrying**: 50-200 mA per bump (signal), >500 mA (power)

### 6.3 RDL Electrical Properties
**Resistance**: 0.05-0.5 Ω/mm (depending on line width and thickness)
**Capacitance**: 0.1-0.3 pF/mm (trace-to-trace)
**Inductance**: 0.3-1 nH/mm
**Skin depth**: ~2 μm at 10 GHz in copper

## 7. Design Rules

### 7.1 Signal Routing
**Differential pairs**: Match lengths to ±100 μm
**Trace width**: Maintain constant width for impedance control
**Via transitions**: Minimize via stubs, use back-drilling if needed
**Reference planes**: Maintain continuous reference for critical signals

### 7.2 Power Distribution
**Plane spacing**: Minimize spacing between power and ground planes
**Via placement**: Decoupling cap vias close to power pins
**Current density**: <100 mA/μm width for RDL copper (continuous), <200 mA/μm (pulsed)

### 7.3 Grounding
**Ground planes**: Continuous ground planes preferred
**Ground vias**: Multiple ground vias for high-speed signals (via shielding)
**Split planes**: Avoid splitting ground planes under high-speed signals

## 8. Testing and Validation

### 8.1 S-Parameter Measurement
**Equipment**: Vector Network Analyzer (VNA) with appropriate frequency range
**Test structures**: Through, short, open, load (SOLT) calibration structures
**Measurement**: 2-port or 4-port S-parameters to 2× maximum operating frequency

### 8.2 Time-Domain Analysis
**TDR (Time Domain Reflectometry)**: Measure impedance vs. position
**TDT (Time Domain Transmission)**: Measure signal propagation
**Eye Diagrams**: Validate signal integrity at receiver

### 8.3 Power Integrity Testing
**PDN impedance**: Measure using VNA in shunt-through configuration
**Voltage ripple**: Measure using oscilloscope at die power pins
**IR drop**: Calculate from current draw and resistance measurements

## 9. Material Electrical Properties

### 9.1 Conductor Materials
**Copper**: Resistivity 1.7 μΩ·cm at 20°C, temperature coefficient +0.4%/°C
**Aluminum**: Resistivity 2.7 μΩ·cm at 20°C
**Solder (SAC305)**: Resistivity 11-15 μΩ·cm

### 9.2 Dielectric Materials
**Silicon Dioxide (SiO₂)**: εr = 3.9, tan δ < 0.001
**Low-k dielectrics**: εr = 2.5-3.0, tan δ = 0.001-0.01
**Polyimide**: εr = 3.2-3.5, tan δ = 0.002-0.008
**BT resin**: εr = 3.3-4.0, tan δ = 0.01-0.02
**FR-4**: εr = 4.2-4.8, tan δ = 0.015-0.025

## 10. Electromagnetic Compatibility (EMC)

### 10.1 Emissions
**Radiated emissions**: Meet FCC Part 15, CISPR 32 limits
**Conducted emissions**: Meet applicable standards
**Mitigation**: Proper grounding, shielding, filtering

### 10.2 Susceptibility
**Radiated susceptibility**: Maintain functionality under specified RF field strengths
**ESD susceptibility**: Meet HBM, CDM requirements
**Mitigation**: Robust design, guard bands on critical parameters

## 11. Reliability Considerations

### 11.1 Electromigration
**Current density limits**: 
- Aluminum: 1-5 mA/μm² (100°C, 10-year lifetime)
- Copper: 5-20 mA/μm² (100°C, 10-year lifetime)
**Black's Equation**: MTTF = A × J⁻ⁿ × exp(Ea/kT)

### 11.2 Time-Dependent Dielectric Breakdown (TDDB)
**Electric field limits**: <4 MV/cm for SiO₂ at 125°C for 10-year lifetime
**Screening**: HTOL (High Temperature Operating Life) testing

### 11.3 Bias Temperature Instability (BTI)
**Threshold voltage shift**: <50 mV over 10-year lifetime at operating conditions
**NBTI (Negative Bias Temperature Instability)**: Affects PMOS transistors
**PBTI (Positive Bias Temperature Instability)**: Affects NMOS transistors

## 12. Compliance

Electrical compliance demonstrated through:
- S-parameter measurements meeting specifications
- Power integrity simulations and measurements
- ESD qualification per JEDEC standards
- EMC testing per applicable standards
- Reliability qualification (HTOL, thermal cycling, etc.)

---

**© 2025 SmileStory Inc. / WIA**
弘益人間 (홍익인간) · Benefit All Humanity
EOF

# Create reliability-spec.md
cat > WIA-SEMI-020-reliability-spec.md << 'EOF'
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
EOF

chmod +x create_specs.sh
./create_specs.sh
ls -lh *.md
