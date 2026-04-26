# WIA-COMM-007: Optical Communication Specification v1.0

> **Standard ID:** WIA-COMM-007
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Communication Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Fiber Optic Fundamentals](#2-fiber-optic-fundamentals)
3. [Optical Transmitters](#3-optical-transmitters)
4. [Optical Receivers](#4-optical-receivers)
5. [Wavelength Division Multiplexing](#5-wavelength-division-multiplexing)
6. [Optical Amplifiers](#6-optical-amplifiers)
7. [Coherent Optical Transmission](#7-coherent-optical-transmission)
8. [Optical Transceivers](#8-optical-transceivers)
9. [Dispersion and Compensation](#9-dispersion-and-compensation)
10. [Optical Switching and ROADM](#10-optical-switching-and-roadm)
11. [Free-Space Optical Communication](#11-free-space-optical-communication)
12. [Submarine Cable Systems](#12-submarine-cable-systems)
13. [Performance Monitoring](#13-performance-monitoring)
14. [Implementation Guidelines](#14-implementation-guidelines)
15. [References](#15-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for optical communication systems, covering fiber optic transmission, wavelength division multiplexing, coherent detection, and high-capacity network architectures.

### 1.2 Scope

The standard covers:
- Single-mode and multi-mode fiber transmission
- WDM and DWDM systems (100+ channels)
- Coherent optical transmission with advanced modulation
- Optical amplification (EDFA, Raman)
- Transceiver modules (SFP to 800G)
- Dispersion management
- Optical switching (ROADM)
- Free-space optical links
- Submarine cable systems
- Signal regeneration and monitoring

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to advance optical communication technology for global connectivity, enabling high-bandwidth internet access, cloud computing, and digital transformation worldwide.

### 1.4 Terminology

- **SMF**: Single-Mode Fiber
- **MMF**: Multi-Mode Fiber
- **DWDM**: Dense Wavelength Division Multiplexing
- **EDFA**: Erbium-Doped Fiber Amplifier
- **OSNR**: Optical Signal-to-Noise Ratio
- **BER**: Bit Error Rate
- **FEC**: Forward Error Correction
- **ROADM**: Reconfigurable Optical Add-Drop Multiplexer
- **FSO**: Free-Space Optical Communication

---

## 2. Fiber Optic Fundamentals

### 2.1 Single-Mode Fiber (SMF)

#### Core Structure
- **Core diameter**: 8-10 μm
- **Cladding diameter**: 125 μm
- **Core index**: n₁ ≈ 1.4681
- **Cladding index**: n₂ ≈ 1.4628

#### Numerical Aperture
```
NA = √(n₁² - n₂²) ≈ 0.13
```

#### Cutoff Wavelength
For single-mode operation:
```
λ_cutoff = 2πa(NA) / 2.405
```

Typical: 1100-1280 nm for standard SMF

### 2.2 Fiber Attenuation

| Wavelength | Attenuation | Primary Mechanism |
|------------|-------------|-------------------|
| 850 nm | 2.5 dB/km | Rayleigh scattering |
| 1310 nm | 0.35 dB/km | Minimum dispersion |
| 1550 nm | 0.20 dB/km | Minimum attenuation |
| 1625 nm | 0.23 dB/km | L-band |

#### Total Loss Budget
```
L_total = α × L + N_c × L_c + N_s × L_s
```

Where:
- `α` = Fiber attenuation (dB/km)
- `L` = Fiber length (km)
- `N_c` = Number of connectors
- `L_c` = Connector loss (0.3-0.5 dB)
- `N_s` = Number of splices
- `L_s` = Splice loss (0.05-0.1 dB)

### 2.3 Multi-Mode Fiber (MMF)

#### Core Sizes
- **OM1**: 62.5 μm core, 850 nm
- **OM2**: 50 μm core, 850 nm
- **OM3**: 50 μm core, 850 nm (laser-optimized)
- **OM4**: 50 μm core, 850 nm (higher bandwidth)
- **OM5**: 50 μm core, 850 nm + SWDM

#### Modal Bandwidth
```
BW × L = constant
```

Typical OM4: 4700 MHz·km @ 850 nm

### 2.4 Specialty Fibers

#### Dispersion-Shifted Fiber (DSF)
- Zero dispersion @ 1550 nm
- Enables dense WDM

#### Non-Zero Dispersion-Shifted Fiber (NZDSF)
- Small dispersion: +2 to +6 ps/(nm·km)
- Reduces four-wave mixing

#### Photonic Crystal Fiber (PCF)
- Air-hole cladding
- Endlessly single-mode
- Low nonlinearity

---

## 3. Optical Transmitters

### 3.1 Laser Diodes

#### Distributed Feedback (DFB) Laser
```
λ_Bragg = 2n_eff Λ
```

**Characteristics**:
- Wavelength: 1310 nm, 1550 nm
- Linewidth: 1-10 MHz
- Side-mode suppression: >30 dB
- Output power: 0-10 dBm

#### Vertical-Cavity Surface-Emitting Laser (VCSEL)
- Wavelength: 850 nm, 1310 nm
- Low power consumption
- High-speed modulation: 25+ Gbps
- Applications: Short-reach datacenter

### 3.2 Optical Modulators

#### Mach-Zehnder Modulator (MZM)
Transfer function:
```
T = 0.5[1 + cos(πV/V_π)]
```

**Parameters**:
- `V_π`: 3-6 V
- Bandwidth: 40+ GHz
- Extinction ratio: >20 dB

#### Electro-Absorption Modulator (EAM)
- Compact integration with DFB
- Lower V_π: 1-2 V
- Chirp: negative (advantageous)

### 3.3 Direct Modulation

**Chirp parameter**:
```
α_chirp = 4π × Δν × Δt
```

Where:
- `Δν` = Frequency deviation
- `Δt` = Pulse width

Typical: α_chirp = 2-6 for DFB lasers

---

## 4. Optical Receivers

### 4.1 PIN Photodiode

#### Responsivity
```
R = η(eλ)/(hc)
```

For λ = 1550 nm, η = 0.9:
```
R ≈ 1.12 A/W
```

#### Receiver Sensitivity

For BER = 10⁻⁹:
```
P_min = (Q × hν/η) × √(2B)
```

Where:
- `Q` ≈ 6 for BER = 10⁻⁹
- `B` = Bit rate

### 4.2 Avalanche Photodiode (APD)

#### Gain
```
M = 1 / (1 - (V/V_br)^n)
```

Typical APD gain: 10-20

#### Noise
Excess noise factor:
```
F = kM + (1-k)(2 - 1/M)
```

Where `k` is the ionization ratio

### 4.3 Coherent Receiver

**Heterodyne detection**:
```
i(t) ∝ E_signal × E_LO × cos(Δωt + Δφ)
```

**Intradyne detection**:
- Digital signal processing (DSP)
- Carrier recovery in digital domain
- Enables advanced modulation formats

---

## 5. Wavelength Division Multiplexing

### 5.1 CWDM (Coarse WDM)

ITU-T G.694.2 grid:
- Wavelengths: 1271-1611 nm
- Spacing: 20 nm
- Channels: 18
- Applications: Metro networks, 10 km range

### 5.2 DWDM (Dense WDM)

ITU-T G.694.1 grid:
- Reference: 193.1 THz (1552.52 nm)
- Spacing: 100, 50, 25, 12.5 GHz
- C-band: 1530-1565 nm (96 channels @ 50 GHz)
- L-band: 1565-1625 nm (additional 80 channels)

#### Channel Frequency
```
f_n = 193.1 + n × Δf
```

Where:
- `n` = Channel number
- `Δf` = Channel spacing (THz)

#### Total Capacity
```
C_total = N × R
```

Example: 96 channels × 100 Gbps = 9.6 Tbps

### 5.3 WDM Components

#### Arrayed Waveguide Grating (AWG)
- Channels: 40, 80, 96
- Insertion loss: 3-5 dB
- Crosstalk: <-30 dB
- Passband: ±0.1 nm

#### Thin-Film Filter (TFF)
- Low insertion loss: 0.5-1.5 dB
- High isolation: >40 dB
- Compact packaging

#### Fiber Bragg Grating (FBG)
Bragg condition:
```
λ_B = 2n_eff Λ
```

Applications: Channel filtering, dispersion compensation

---

## 6. Optical Amplifiers

### 6.1 Erbium-Doped Fiber Amplifier (EDFA)

#### Gain Spectrum
- C-band: 1530-1565 nm
- L-band: 1565-1625 nm

#### Gain Equation
```
G = exp[(σ_e N_2 - σ_a N_1) × L]
```

Where:
- `σ_e` = Emission cross-section
- `σ_a` = Absorption cross-section
- `N_1, N_2` = Ground and excited state populations
- `L` = Fiber length

#### Noise Figure
```
NF = 2n_sp(G-1)/G
```

Where:
- `n_sp` = Spontaneous emission factor
- Typical: NF = 4-6 dB for EDFA

#### Amplified Spontaneous Emission (ASE)
```
P_ASE = 2n_sp hν(G-1)B_o
```

### 6.2 Raman Amplifier

#### Raman Gain
Peak gain at Stokes shift: ~13 THz

```
g_R ≈ 6 × 10⁻¹⁴ m/W (peak)
```

**Advantages**:
- Distributed amplification
- Lower noise figure
- Flexible gain spectrum

**Pump wavelengths** (for 1550 nm signal):
- 1450-1480 nm

### 6.3 Semiconductor Optical Amplifier (SOA)

- Gain: 15-25 dB
- Noise figure: 6-9 dB
- Gain bandwidth: >60 nm
- Fast gain dynamics: ns
- Applications: Metro networks, packet switching

---

## 7. Coherent Optical Transmission

### 7.1 Modulation Formats

#### Differential Phase-Shift Keying (DPSK)
```
Δφ = 0, π (DBPSK)
Δφ = 0, π/2, π, 3π/2 (DQPSK)
```

**OSNR requirement**: ~3 dB better than OOK

#### Dual-Polarization QPSK (DP-QPSK)
- 4 bits/symbol
- Spectral efficiency: 2 bit/s/Hz
- Standard for 100G systems

#### DP-16QAM
- 8 bits/symbol
- Spectral efficiency: 4 bit/s/Hz
- OSNR requirement: ~7 dB higher than DP-QPSK
- Applications: Metro 200G/400G

#### DP-64QAM
- 12 bits/symbol
- Spectral efficiency: 6 bit/s/Hz
- OSNR requirement: ~14 dB higher than DP-QPSK
- Applications: DCI, short reach

### 7.2 Digital Signal Processing

#### DSP Functions
1. **Chromatic dispersion compensation**
2. **Polarization de-multiplexing**
3. **Carrier recovery**
4. **Phase estimation**
5. **FEC decoding**

#### Dispersion Compensation Filter
Frequency domain:
```
H(ω) = exp(jβ₂ω²L/2)
```

Where:
- `β₂` = Dispersion parameter
- `L` = Fiber length

### 7.3 Optical Signal-to-Noise Ratio (OSNR)

```
OSNR = P_signal / P_ASE
```

Measured in 0.1 nm bandwidth

**Required OSNR** (for BER = 10⁻³, pre-FEC):
- DP-QPSK: 11-14 dB
- DP-16QAM: 18-21 dB
- DP-64QAM: 25-28 dB

---

## 8. Optical Transceivers

### 8.1 Form Factors

#### Small Form-Factor Pluggable (SFP)
- Data rate: 1-10 Gbps (SFP+)
- Reach: 300 m to 80 km
- Wavelengths: 850, 1310, 1550 nm
- Power: <1.5 W

#### Quad SFP (QSFP)
- **QSFP**: 4×10G = 40G
- **QSFP28**: 4×25G = 100G
- **QSFP56**: 4×50G = 200G
- **QSFP-DD**: 8×50G = 400G
- **QSFP-DD800**: 8×100G = 800G

#### C Form-Factor Pluggable (CFP)
- CFP2-DCO: 100G/200G coherent
- CFP2-ACO: 400G coherent
- Integrated DSP and laser

### 8.2 IEEE 802.3 Ethernet Standards

| Standard | Data Rate | Fiber Type | Wavelength | Reach |
|----------|-----------|------------|------------|-------|
| 1000BASE-SX | 1 Gbps | MMF | 850 nm | 550 m |
| 1000BASE-LX | 1 Gbps | SMF | 1310 nm | 10 km |
| 10GBASE-SR | 10 Gbps | MMF | 850 nm | 300 m |
| 10GBASE-LR | 10 Gbps | SMF | 1310 nm | 10 km |
| 40GBASE-SR4 | 40 Gbps | MMF | 850 nm | 100 m |
| 100GBASE-SR4 | 100 Gbps | MMF | 850 nm | 100 m |
| 100GBASE-LR4 | 100 Gbps | SMF | CWDM4 | 10 km |
| 100GBASE-ER4 | 100 Gbps | SMF | CWDM4 | 40 km |
| 400GBASE-DR4 | 400 Gbps | SMF | 1310 nm | 500 m |
| 400GBASE-FR4 | 400 Gbps | SMF | CWDM4 | 2 km |
| 400GBASE-LR8 | 400 Gbps | SMF | LWDM8 | 10 km |

### 8.3 Transceiver Specifications

#### Key Parameters
- **TX power**: -10 to +5 dBm
- **RX sensitivity**: -20 to -28 dBm
- **Overload**: -3 dBm
- **Extinction ratio**: >8 dB
- **Eye mask**: IEEE 802.3 compliance
- **BER**: <10⁻¹²

---

## 9. Dispersion and Compensation

### 9.1 Chromatic Dispersion

#### Dispersion Parameter
```
D(λ) = -(λ/c) × d²n/dλ²
```

**Standard SMF**:
- D(1310 nm) ≈ 0 ps/(nm·km)
- D(1550 nm) ≈ +17 ps/(nm·km)

#### Pulse Broadening
```
Δτ = |D| × Δλ × L
```

Where:
- `Δλ` = Spectral width
- `L` = Fiber length

### 9.2 Dispersion Compensation

#### Dispersion Compensating Fiber (DCF)
- D ≈ -80 to -100 ps/(nm·km)
- Used to compensate SMF dispersion
- Loss: 0.5 dB/km

#### Fiber Bragg Grating (FBG)
- Chirped grating
- Compact
- Low loss: 1-3 dB
- Bandwidth: 0.2-1 nm

#### Digital Dispersion Compensation
In coherent systems:
```
H(ω) = exp(-jβ₂ω²L/2)
```

Implemented in DSP

### 9.3 Polarization Mode Dispersion (PMD)

#### PMD Accumulation
```
⟨Δτ⟩ = D_PMD × √L
```

**Modern fiber**: D_PMD < 0.1 ps/√km

#### PMD Tolerance
For 10 Gbps: PMD < 10% of bit period = 10 ps
For 100 Gbps: PMD < 1 ps

---

## 10. Optical Switching and ROADM

### 10.1 ROADM Architecture

#### Wavelength Selective Switch (WSS)
- Channels: 96 (50 GHz grid)
- Insertion loss: 5-7 dB
- Switching time: <10 ms
- Port count: 1×9, 1×20

#### ROADM Degrees
- **Degree**: Number of fiber directions
- Common: 2-degree, 4-degree, 8-degree
- Colorless, Directionless, Contentionless (CDC)

### 10.2 Optical Cross-Connect (OXC)

**Switching fabric**:
- MEMS mirrors
- Liquid crystal on silicon (LCoS)
- Wavelength selective switches

**Port count**: 320×320 and larger

### 10.3 Add/Drop Multiplexer

#### Fixed OADM
- Add/drop specific wavelengths
- Passive filters (TFF, FBG)
- No reconfiguration

#### Reconfigurable OADM (ROADM)
- Dynamic add/drop
- Remote provisioning
- Mesh network support

---

## 11. Free-Space Optical Communication

### 11.1 FSO Link Budget

```
P_rx = P_tx + G_tx + G_rx - L_geometric - L_atm
```

Where:
- `L_geometric = 20log₁₀(4πd/λ)` (free-space path loss)
- `L_atm` = Atmospheric attenuation

#### Atmospheric Attenuation
```
L_atm(dB/km) = a + b × V^c
```

Where `V` is visibility in km

**Typical**:
- Clear air: 0.2-0.5 dB/km
- Haze: 2-5 dB/km
- Fog: >10 dB/km

### 11.2 Beam Divergence

```
θ = 2.44 × λ/D
```

Where `D` is transmitter aperture

**Example**: λ = 1550 nm, D = 10 cm
```
θ ≈ 38 μrad
```

Beam diameter at 1 km: 3.8 cm

### 11.3 Scintillation

Intensity variance:
```
σ_I² = 1.23 C_n² k^(7/6) L^(11/6)
```

Where:
- `C_n²` = Refractive index structure parameter
- `k` = 2π/λ
- `L` = Link length

**Mitigation**:
- Aperture averaging
- Adaptive optics
- Spatial diversity

---

## 12. Submarine Cable Systems

### 12.1 System Architecture

#### Typical Submarine Link
- **Fiber pairs**: 6-24
- **Repeater spacing**: 40-80 km
- **Capacity per pair**: 20-30 Tbps
- **Total capacity**: 200-400 Tbps
- **Distance**: 6,000-15,000 km

#### Repeater (Amplifier)
- **Type**: EDFA (C+L bands)
- **Gain**: 15-20 dB per band
- **Noise figure**: 4-5 dB
- **Lifetime**: 25+ years

#### Power Feed
- Voltage: ±10 kV DC
- Current: 1-2 A
- Series powering through repeaters

### 12.2 Design Considerations

#### Q-Factor Budget
```
Q = 20log₁₀(√(2SNR))
```

Target Q > 8.5 dB for BER < 10⁻³ (pre-FEC)

#### Amplifier Spacing
Optimal spacing:
```
L_opt = √(2 / (α × γ × P_ch))
```

Where:
- `α` = Fiber loss
- `γ` = Nonlinear coefficient
- `P_ch` = Channel power

### 12.3 Examples

| Cable | Year | Route | Distance | Capacity | Fiber Pairs |
|-------|------|-------|----------|----------|-------------|
| TAT-14 | 2001 | Transatlantic | 15,000 km | 3.2 Tbps | 4 |
| FASTER | 2016 | Trans-Pacific | 11,600 km | 60 Tbps | 6 |
| MAREA | 2018 | Transatlantic | 6,600 km | 200 Tbps | 8 |
| Dunant | 2020 | Transatlantic | 6,400 km | 250 Tbps | 12 |
| 2Africa | 2024 | Around Africa | 45,000 km | 180 Tbps | 16 |

---

## 13. Performance Monitoring

### 13.1 Optical Performance Monitoring (OPM)

#### Key Metrics
- **Optical power**: dBm
- **OSNR**: dB (0.1 nm bandwidth)
- **Chromatic dispersion**: ps/nm
- **PMD**: ps
- **Wavelength**: nm (±0.01 nm)

#### Measurement Techniques
- Optical spectrum analyzer (OSA)
- Polarization analyzer
- Coherent detection
- Asynchronous sampling

### 13.2 Bit Error Rate Testing

#### BER Measurement
```
BER = N_errors / N_total
```

**Standards**:
- FEC threshold: 10⁻³ to 10⁻⁴
- Post-FEC: <10⁻¹⁵

#### Eye Diagram
- Amplitude: Eye height
- Timing: Eye width
- Noise: Eye closure
- Q-factor: Eye quality

### 13.3 Forward Error Correction

#### Reed-Solomon Codes
- RS(255,239): 7% overhead, 5.8 dB gain
- RS(528,514): 2.7% overhead, 4.5 dB gain

#### Soft-Decision FEC (SD-FEC)
- Overhead: 20-27%
- Net coding gain: 10-12 dB
- Applications: Long-haul coherent

#### Low-Density Parity-Check (LDPC)
- Overhead: 25%
- NCG: 11-12 dB
- IEEE 802.3 400G/800G

---

## 14. Implementation Guidelines

### 14.1 Link Design Procedure

1. **Define requirements**:
   - Distance
   - Data rate
   - Number of channels
   - Availability target

2. **Select fiber type**:
   - SMF for long haul
   - MMF for short reach
   - Specialty fiber as needed

3. **Calculate power budget**:
   - TX power
   - RX sensitivity
   - Fiber loss
   - Component losses
   - Margin: 3-6 dB

4. **Verify dispersion**:
   - Chromatic dispersion tolerance
   - PMD tolerance
   - Compensation if needed

5. **Check nonlinearities**:
   - Self-phase modulation
   - Cross-phase modulation
   - Four-wave mixing

6. **Select amplifiers**:
   - Span loss compensation
   - OSNR accumulation
   - Noise figure budget

### 14.2 Network Architecture

#### Point-to-Point
- Simplest topology
- Direct fiber connection
- Applications: Metro, leased lines

#### Ring
- Bidirectional
- Protection switching
- SONET/SDH, OTN

#### Mesh
- ROADM-based
- Any-to-any connectivity
- Dynamic provisioning
- Resilience

### 14.3 Testing and Commissioning

#### Fiber Testing
- **OTDR** (Optical Time-Domain Reflectometer):
  - Fiber length
  - Splice/connector loss
  - Fault location

- **Light source + power meter**:
  - End-to-end loss
  - Insertion loss

#### System Testing
- BER testing (pre/post FEC)
- OSNR measurement
- Dispersion measurement
- PMD measurement
- Eye diagram analysis

---

## 15. References

### 15.1 Standards

- ITU-T G.652: Single-mode optical fiber
- ITU-T G.655: Non-zero dispersion-shifted fiber
- ITU-T G.694.1: DWDM frequency grid
- ITU-T G.698.2: Multi-channel DWDM applications
- IEEE 802.3: Ethernet standards
- OIF (Optical Internetworking Forum): Implementation agreements

### 15.2 Textbooks

- Agrawal, "Fiber-Optic Communication Systems"
- Ramaswami, Sivarajan, Sasaki, "Optical Networks"
- Keiser, "Optical Fiber Communications"
- Saleh & Teich, "Fundamentals of Photonics"

### 15.3 Online Resources

- ITU-T Recommendations: [itu.int](https://www.itu.int)
- IEEE Standards: [ieee.org](https://www.ieee.org)
- OIF: [oiforum.com](https://www.oiforum.com)
- TeleGeography Submarine Cable Map: [submarinecablemap.com](https://www.submarinecablemap.com)

---

## Appendix A: Common Wavelengths

| Application | Wavelength | Band | Typical Use |
|-------------|------------|------|-------------|
| Multi-mode datacenter | 850 nm | O-band | 10G/25G/100G SR |
| Metro/access | 1310 nm | O-band | 10G/100G LR |
| Long-haul DWDM | 1530-1565 nm | C-band | 100G/200G/400G |
| Extended DWDM | 1565-1625 nm | L-band | Additional capacity |
| CWDM | 1271-1611 nm | O/E/S/C/L | Coarse WDM |

---

## Appendix B: Loss Budget Calculator

```
Link Budget Calculation:

TX Power:           P_tx (dBm)
RX Sensitivity:     P_rx (dBm)
Required Margin:    M (dB)

Available Budget:   P_tx - P_rx - M

Losses:
- Fiber:            α × L (dB)
- Connectors:       N_c × 0.5 (dB)
- Splices:          N_s × 0.1 (dB)
- Other:            L_other (dB)

Total Loss:         L_total

Link Margin:        (P_tx - P_rx) - L_total

Status:             Pass if margin ≥ M
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
