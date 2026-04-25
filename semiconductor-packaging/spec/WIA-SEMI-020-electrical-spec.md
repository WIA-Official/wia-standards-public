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
