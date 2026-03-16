# WIA-SEMI-006: Optical Specifications for Photonic Chips

**Version:** 1.0
**Date:** 2025-01-15
**Status:** Published
**Editor:** World Certification Industry Association (WIA)

---

## 1. Scope

This specification defines the optical performance requirements, measurement methodologies, and compliance criteria for photonic integrated circuits (PICs) used in data center interconnects, telecommunications, and high-performance computing applications. It covers wavelength ranges from 1260 nm to 1625 nm (O-band through L-band) and data rates from 100 Gbps to 1.6 Tbps per fiber.

## 2. Normative References

- **ITU-T G.694.1:** Spectral grids for WDM applications
- **IEEE 802.3:** Ethernet standards (400GBASE, 800GBASE)
- **IEC 61300-3:** Fiber optic interconnecting devices and passive components
- **ISO 9001:2015:** Quality management systems

## 3. Optical Waveguide Specifications

### 3.1 Strip Waveguide Geometry

**Standard Dimensions:**
- Width: 450 nm ± 10 nm
- Height: 220 nm ± 5 nm
- Sidewall angle: 90° ± 3°
- Surface roughness: < 2 nm RMS

**Optical Properties:**
- Effective index (TE mode, λ = 1550 nm): 2.45 ± 0.05
- Propagation loss: < 3 dB/cm at 1550 nm
- Bend radius (90° turn): ≥ 5 μm for loss < 0.05 dB per bend
- Group index: 4.2 ± 0.1

### 3.2 Rib Waveguide Geometry

**Standard Dimensions:**
- Rib width: 500-1000 nm
- Rib height: 150-180 nm
- Slab thickness: 40-70 nm

**Performance:**
- Propagation loss: < 1.5 dB/cm
- Single-mode operation: Ensured for rib widths < 900 nm

### 3.3 Silicon Nitride (SiN) Waveguides

**Material System:**
- Si₃N₄ thickness: 400-800 nm
- SiO₂ lower cladding: ≥ 2 μm
- Refractive index: 2.0 ± 0.02 at 1550 nm

**Ultra-Low Loss Target:**
- Propagation loss: < 0.5 dB/cm (standard)
- Propagation loss: < 0.1 dB/cm (premium)

---

## 4. Wavelength Division Multiplexing (WDM)

### 4.1 Channel Grid

**ITU-T Compliance:**
- Anchor frequency: 193.1 THz (1552.52 nm)
- Channel spacing: 50 GHz, 100 GHz, or 200 GHz
- Frequency accuracy: ± 2.5 GHz

**Wavelength Bands:**
| Band | Wavelength Range | Typical Application |
|------|------------------|---------------------|
| O-Band | 1260-1360 nm | Short-reach data center |
| E-Band | 1360-1460 nm | Extended reach |
| S-Band | 1460-1530 nm | CWDM applications |
| C-Band | 1530-1565 nm | Long-haul DWDM |
| L-Band | 1565-1625 nm | Ultra-long-haul |

### 4.2 Multiplexer/Demultiplexer Performance

**Arrayed Waveguide Grating (AWG):**
- Insertion loss: < 4 dB
- Adjacent channel crosstalk: < -25 dB
- Non-adjacent channel crosstalk: < -35 dB
- Passband width (1 dB): ≥ 0.4 nm for 100 GHz spacing
- Polarization dependent loss (PDL): < 1 dB
- Temperature sensitivity: 0.01 nm/°C (typical)

**Ring Resonator Filters:**
- Insertion loss (on-resonance): < 1 dB
- Extinction ratio (off-resonance): > 20 dB
- Quality factor (Q): > 10,000
- Free spectral range (FSR): Matched to channel spacing
- Tuning range: > 10 nm with thermal heaters

---

## 5. Fiber Coupling Interfaces

### 5.1 Edge Coupling

**Spot Size Converter (SSC):**
- Mode field diameter: 3-5 μm (output)
- Coupling loss: < 1.5 dB per facet (with AR coating)
- Bandwidth: > 100 nm
- Polarization dependent loss: < 0.5 dB

**Anti-Reflection (AR) Coating:**
- Reflectivity: < -40 dB
- Wavelength range: 1260-1625 nm

### 5.2 Grating Couplers

**Standard Grating Coupler:**
- Peak coupling efficiency: > -4 dB
- 1-dB bandwidth: > 30 nm
- Optimal fiber angle: 8-10° from vertical
- Polarization: TE or TM optimized (single-polarization)
- Alignment tolerance: ± 1 μm lateral, ± 5 μm vertical

**Advanced Grating Coupler (with bottom mirror):**
- Peak coupling efficiency: > -1.5 dB
- 1-dB bandwidth: > 80 nm
- Back-reflection: < -25 dB

---

## 6. Optical Loss Budget

### 6.1 Maximum Allowable Loss

**Short-Reach (SR) Links (< 100 m):**
- Total link loss budget: 3-5 dB
- Components:
  - Fiber coupling (2×): 3 dB
  - Waveguide propagation: 1 dB
  - Splitters/combiners: 0.5 dB
  - Margin: 0.5 dB

**Medium-Reach (DR) Links (500 m):**
- Total link loss budget: 6-8 dB

**Long-Reach (FR) Links (2 km):**
- Total link loss budget: 8-12 dB

### 6.2 Component Loss Specifications

| Component | Maximum Loss |
|-----------|--------------|
| Straight waveguide | 3 dB/cm |
| 90° waveguide bend (R=5μm) | 0.05 dB |
| Y-branch splitter (1×2) | 0.3 dB |
| MMI coupler (2×2) | 0.5 dB |
| AWG multiplexer | 4 dB |
| Mach-Zehnder modulator | 6 dB |
| Ring modulator | 2 dB |
| Photodetector (coupling) | 1 dB |

---

## 7. Spectral Characteristics

### 7.1 Laser Source Requirements

**Wavelength Stability:**
- Temperature drift: < 0.1 nm/°C
- Long-term drift: < 0.5 nm over 15 years
- Side-mode suppression ratio (SMSR): > 30 dB

**Linewidth:**
- Direct modulation: < 10 MHz
- External modulation: < 1 MHz
- Coherent applications: < 100 kHz

### 7.2 Modulator Spectral Purity

**Chirp Parameter (α):**
- MZM: |α| < 0.3
- Ring modulator: |α| < 0.5

**Spurious Mode Suppression:**
- Higher-order harmonics: < -20 dBc

---

## 8. Polarization Management

### 8.1 Polarization Dependence

**Maximum Allowed PDL:**
- Passive components: < 0.5 dB
- Modulators: < 1.0 dB
- Total link: < 2.0 dB

### 8.2 Polarization Extinction Ratio (PER)

**TE/TM Isolation:**
- Waveguides: > 20 dB
- Polarization rotators: > 15 dB

---

## 9. Environmental and Reliability

### 9.1 Operating Conditions

**Temperature Range:**
- Standard: 0°C to 70°C
- Extended: -5°C to 85°C

**Humidity:**
- Relative humidity: 5% to 85% non-condensing

### 9.2 Wavelength Temperature Coefficient

**Allowable Drift:**
- Silicon devices: 0.08-0.10 nm/°C
- Athermal designs: < 0.01 nm/°C

### 9.3 Lifetime Requirements

**Mean Time Between Failures (MTBF):**
- Data center modules: > 1,000,000 hours at 40°C
- Telecom modules: > 2,000,000 hours at 25°C

**Accelerated Aging:**
- HTOL testing: 2000 hours at 85°C
- Maximum degradation: 1 dB optical power, 2× BER increase

---

## 10. Measurement Standards

### 10.1 Insertion Loss Measurement

**Method:**
- Substitution method per IEC 61300-3-4
- Calibrated broadband source + power meter
- Measurement uncertainty: ± 0.1 dB

### 10.2 Spectral Measurement

**Equipment:**
- Optical spectrum analyzer (OSA)
- Resolution bandwidth: 0.01-0.1 nm
- Dynamic range: > 60 dB

### 10.3 Time-Domain Measurements

**Eye Diagram Analysis:**
- Oscilloscope bandwidth: ≥ 2× data rate
- Sampling rate: ≥ 4× symbol rate
- Metrics: Eye height, eye width, jitter (RMS and pk-pk)

---

## 11. Compliance Testing

### 11.1 Type Testing

Required for new designs and process changes:
- Full optical characterization across temperature and wavelength
- Accelerated reliability testing (HTOL, thermal cycling, humidity)
- Interoperability testing with reference transceivers

### 11.2 Production Testing

Required for every production unit:
- Optical power output
- Insertion loss
- BER testing at nominal and minimum optical power
- Temperature cycling (subset of units)

### 11.3 Field Testing

Periodic sampling in deployment:
- Long-term stability monitoring
- Environmental stress screening
- Failure analysis and corrective action

---

## 12. Marking and Documentation

### 12.1 Product Marking

Each compliant device shall be marked with:
- WIA-SEMI-006 compliance logo
- Manufacturer name and part number
- Date code (YYWW format)
- Wavelength range or channel designation

### 12.2 Documentation Requirements

Manufacturers must provide:
- Datasheet with all specified parameters
- Test reports demonstrating compliance
- Assembly and handling instructions
- Reliability data and MTBF calculations

---

## 13. Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01-15 | Initial release |

---

**Published by:**
World Certification Industry Association (WIA)
弘益人間 (Hongik Ingan) - Benefit All Humanity

© 2025 WIA. All rights reserved.
