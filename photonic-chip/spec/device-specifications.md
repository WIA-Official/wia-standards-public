# WIA-SEMI-006: Device Specifications for Active Photonic Components

**Version:** 1.0
**Date:** 2025-01-15
**Editor:** World Certification Industry Association (WIA)

---

## 1. High-Speed Modulators

### 1.1 Mach-Zehnder Modulators (MZM)

**Electro-Optic Performance:**
- Bandwidth (-3dB electrical-to-optical): ≥ 50 GHz
- Vπ × Lπ figure of merit: < 2.5 V·cm
- Drive voltage (Vπ): 2-4 Vpp differential
- Extinction ratio (ER): > 6 dB (static), > 4 dB (dynamic at max speed)
- Insertion loss: < 6 dB (includes coupling and propagation)

**Modulation Formats Supported:**
- NRZ: up to 100 Gbps
- PAM4: up to 200 Gbps
- PAM8: up to 300 Gbps (with DSP equalization)

**Electrical Interface:**
- Impedance: 50Ω differential
- Return loss (S11): > 10 dB up to 60 GHz
- RF bandwidth: DC to 70 GHz

**Temperature Performance:**
- Operating range: -5°C to 75°C
- Vπ temperature coefficient: < 5% over temperature range
- Bias point drift: < λ/20 over 24 hours at constant temperature

---

### 1.2 Ring Resonator Modulators

**Resonator Characteristics:**
- Free spectral range (FSR): 10-40 nm (matched to WDM grid)
- Quality factor (Q): 10,000 - 50,000
- Extinction ratio: > 10 dB
- Insertion loss (through port): < 2 dB

**Tuning:**
- Thermal tuning range: > 10 nm
- Thermal tuning efficiency: 15-30 mW/nm
- Tuning speed: 1-10 μs (thermal)
- Carrier tuning speed: < 100 ps (with pre-emphasis)

**Energy Efficiency:**
- Energy per bit: < 100 fJ/bit at 50 Gbps
- Static power (thermal stabilization): < 50 mW per ring

---

### 1.3 Traveling Wave Modulators

**Performance:**
- Bandwidth: > 100 GHz
- Vπ × Lπ: < 2 V·cm
- Velocity mismatch: < 10%
- Microwave loss: < 1 dB/mm at 50 GHz

**Design Requirements:**
- Electrode design: Coplanar waveguide (CPW) or microstrip
- Velocity matching: Index matching via ground plane design
- Termination: On-chip 50Ω resistor or off-chip termination

---

## 2. Photodetectors

### 2.1 Germanium (Ge) Waveguide Photodetectors

**Optical Performance:**
- Responsivity: > 0.8 A/W at 1550 nm, > 0.6 A/W at 1310 nm
- Bandwidth (-3dB): ≥ 50 GHz
- Dark current: < 100 nA at -1V bias
- Saturation current: > 10 mA

**Geometry:**
- Ge absorption length: 10-30 μm
- Ge thickness: 500-1000 nm
- Junction type: Vertical p-i-n or lateral p-i-n
- Contact configuration: Separate absorption and detection (optional)

**Capacitance:**
- Junction capacitance: < 20 fF
- RC-limited bandwidth: > 50 GHz

**Temperature Stability:**
- Dark current doubling temperature: > 7°C
- Responsivity temperature coefficient: < 0.1% / °C

---

### 2.2 Avalanche Photodiodes (APD)

**Ge/Si Separate Absorption and Multiplication (SAM) APD:**
- Gain: 5-20 (adjustable via bias voltage)
- Gain-bandwidth product: > 340 GHz
- Excess noise factor (F): < 5 at M=10
- Dark current at M=10: < 1 μA
- Responsivity (M=1): > 0.9 A/W at 1550 nm

**Bias Voltage:**
- Operating voltage: 15-30 V
- Voltage for M=10: Typically 25-28 V

---

## 3. Optical Switches

### 3.1 Thermo-Optic Switches

**Switching Performance:**
- Extinction ratio: > 25 dB
- Insertion loss: < 1.5 dB (bar state), < 1.5 dB (cross state)
- Switching time: 1-10 μs
- Power consumption: 10-50 mW per switch element

**Crosstalk:**
- Adjacent channel: < -30 dB
- Non-adjacent channel: < -40 dB

**Scalability:**
- Switch matrix size: Up to 8×8 demonstrated, larger matrices via cascading

---

### 3.2 Carrier-Injection Switches

**Performance:**
- Switching time: 100 ns - 1 μs
- Extinction ratio: > 20 dB
- Drive current: 10-50 mA
- Forward voltage: 0.7-1.2 V

---

## 4. Optical Phase Shifters

### 4.1 Thermo-Optic Phase Shifters

**Specifications:**
- Phase shift efficiency: π/mW (power required for π phase shift)
- Typical power for π shift: 10-30 mW
- Response time: 1-10 μs
- Linearity: < 5% deviation from linear over 2π range

**Heater Design:**
- Material: TiN, NiCr, or doped silicon
- Resistance: 100-500 Ω
- Heater width: 2-5 μm, positioned 1-2 μm from waveguide

---

### 4.2 Carrier-Based Phase Shifters

**PN Junction (Depletion Mode):**
- Phase shift efficiency: 20-40°/mm at 1V reverse bias
- Bandwidth: > 50 GHz
- Loss penalty: 5-10 dB/cm

**PIN Junction (Injection Mode):**
- Phase shift efficiency: 40-80°/mm at 10 mA forward current
- Bandwidth: 1-10 GHz (limited by carrier lifetime)
- Loss penalty: 3-5 dB/cm

---

## 5. Wavelength Filters

### 5.1 Ring Resonator Filters

**Single Ring:**
- Free spectral range: 10-100 nm
- Quality factor: 10,000 - 100,000
- Drop port extinction: > 20 dB
- Through port suppression at resonance: > 20 dB

**Cascaded Rings (Higher-Order Filters):**
- Number of rings: 2-8
- Passband shape: Flat-top, Butterworth, or Chebyshev
- Out-of-band rejection: > 40 dB

---

### 5.2 Bragg Gratings

**Performance:**
- Reflection bandwidth: 0.5-5 nm (adjustable via apodization)
- Peak reflectivity: > 90%
- Out-of-band transmission: > 95%
- Sidelobe suppression: > 15 dB

**Application:**
- Add-drop filters for WDM
- Dispersion compensators
- Laser cavity reflectors

---

## 6. Variable Optical Attenuators (VOA)

**Attenuation Range:**
- Dynamic range: 0-30 dB
- Resolution: < 0.5 dB steps
- Wavelength independence: ± 0.5 dB across C-band

**Control Mechanisms:**
- Thermo-optic: Mach-Zehnder interferometer with heater
- Carrier-based: P-N junction absorption modulation
- MEMS: Mechanical mirror or shutter (if integrated)

**Response Time:**
- Thermo-optic: 10 μs
- Carrier-based: 1 ns - 1 μs
- MEMS: 100 μs - 10 ms

---

## 7. Laser Integration

### 7.1 Hybrid III-V/Silicon Lasers

**Output Power:**
- CW output (on-chip): > 10 mW
- Fiber-coupled output: > 1 mW

**Wavelength Characteristics:**
- Center wavelength: Per ITU-T grid or application-specific
- Wavelength accuracy: ± 0.1 nm
- Side-mode suppression ratio (SMSR): > 40 dB

**Threshold and Efficiency:**
- Threshold current: < 20 mA
- Slope efficiency: > 0.15 W/A

**Tuning (if applicable):**
- Tuning range: > 40 nm (for tunable lasers)
- Tuning speed: < 100 ns (for Vernier-tuned designs)

**Reliability:**
- MTBF: > 1,000,000 hours at 40°C
- Maximum degradation: 1 dB over 15 years

---

### 7.2 Quantum Dot Lasers on Silicon

**Performance Targets:**
- Threshold current density: < 500 A/cm²
- Operating temperature: Up to 85°C (uncooled)
- Output power: > 5 mW on-chip
- Defect tolerance: Functionality maintained with dislocation density < 10⁸ cm⁻²

---

## 8. Polarization Management Devices

### 8.1 Polarization Rotators

**Performance:**
- Conversion efficiency (TE to TM or vice versa): > 95%
- Insertion loss: < 1 dB
- Bandwidth: > 100 nm
- Polarization extinction ratio: > 15 dB

---

### 8.2 Polarization Splitters/Combiners (PBS/PBC)

**Specifications:**
- Extinction ratio: > 20 dB for both TE and TM ports
- Insertion loss: < 1 dB per polarization
- Bandwidth: > 80 nm

---

## 9. Nonlinear Devices

### 9.1 Four-Wave Mixing (FWM) for Wavelength Conversion

**Efficiency:**
- Conversion efficiency: -10 to -20 dB (pump to converted signal)
- Pump power: 100 mW - 1 W (on-chip)

**Phase Matching:**
- Group velocity dispersion engineering for broadband phase matching
- Dispersion tolerance: ± 5 ps/nm/km

---

### 9.2 Supercontinuum Generation

**Output Spectrum:**
- Spectral span: > 200 nm
- Flatness: ± 5 dB across central 100 nm
- Pump wavelength: Typically 1550 nm
- Pump pulse width: 100 fs - 10 ps

---

## 10. Testing and Qualification

### 10.1 Device-Level Testing

**DC Characterization:**
- I-V curves for modulators and photodetectors
- Optical transmission vs. wavelength for filters and resonators
- Power vs. current (L-I) for lasers

**RF/High-Speed Characterization:**
- S-parameter measurement (S21, S11) for modulators and photodetectors
- Eye diagram quality assessment
- Bit error rate (BER) testing at target data rates

---

### 10.2 Reliability Stress Testing

**Accelerated Life Testing:**
- High-temperature operating life (HTOL): 1000-2000 hours at 85-125°C
- High-temperature storage (HTS): 1000 hours at 150°C (non-powered)
- Temperature cycling: -40°C to +85°C, 500-1000 cycles
- Humidity testing: 85°C / 85% RH, 1000 hours

**Failure Criteria:**
- Optical power degradation: < 1 dB
- Dark current increase: < 2× initial value
- Threshold current increase (lasers): < 20%

---

**Published by:**
World Certification Industry Association (WIA)
弘益人間 (Hongik Ingan) - Benefit All Humanity

© 2025 WIA. All rights reserved.
