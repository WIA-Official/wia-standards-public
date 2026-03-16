# WIA-SEMI-006: Packaging and Integration Specifications

**Version:** 1.0
**Date:** 2025-01-15
**Editor:** World Certification Industry Association (WIA)

---

## 1. Scope

This specification defines packaging, fiber attachment, thermal management, and co-packaged optics (CPO) integration requirements for photonic chips compliant with WIA-SEMI-006 standard. It addresses mechanical interfaces, environmental protection, and system-level integration guidelines.

---

## 2. Package Types and Form Factors

### 2.1 Pluggable Module Form Factors

**Supported Standards:**
- QSFP28 (100G, 4×25G): 13.6 mm (W) × 72 mm (L) × 8.5 mm (H)
- QSFP-DD (200G/400G): 18.4 mm × 89 mm × 8.5 mm
- OSFP (400G/800G): 22.6 mm × 108 mm × 12.4 mm
- QSFP-DD800 (800G): 18.4 mm × 122 mm × 8.5 mm

**Electrical Interface:**
- High-speed differential pairs per SFF-8686 or SFF-8689
- CMIS (Common Management Interface Specification) for control and monitoring
- Power supply: 3.3V ± 5%, 12V ± 5% (for higher power modules)

**Thermal:**
- Maximum power dissipation: 12W (QSFP-DD), 15W (OSFP), 18W (QSFP-DD800)
- Case temperature: < 70°C under maximum ambient (40°C)

---

### 2.2 Co-Packaged Optics (CPO) Form Factors

**On-Package Integration:**
- Photonic die mounted on ASIC package substrate
- Optical I/O at package edge or through substrate
- Electrical traces: < 5 mm from ASIC to photonics

**Substrate Types:**
- Organic (BT resin, ABF)
- Silicon interposer (2.5D integration)
- Glass interposer (optical TSV capable)

**Mechanical Dimensions:**
- CPO package: Compatible with standard ASIC packages (e.g., BGA, LGA)
- Height increase: < 3 mm for optical fiber connector integration

---

## 3. Fiber Attachment and Optical Connectors

### 3.1 Single-Mode Fiber (SMF) Coupling

**Connector Types:**
- LC duplex: For single-channel or low-channel-count applications
- MPO/MTP (12-fiber, 24-fiber): For parallel optics (8×100G, 4×200G)
- CS connector: For high-density CPO applications

**Performance Requirements:**
- Insertion loss: < 0.5 dB per connection
- Return loss: > 45 dB (with APC connector), > 35 dB (with UPC connector)
- Repeatability: < 0.2 dB variation over 100 mating cycles

---

### 3.2 Fiber Alignment Techniques

**Active Alignment:**
- Light is injected into fiber during assembly, output power maximized via precision stages
- Alignment tolerance: ± 0.5 μm (lateral), ± 1 μm (axial)
- Fixture: UV-curable epoxy to permanently bond fiber after alignment

**Passive Alignment:**
- Mechanical features (V-grooves, photonic wire bonds) define fiber position
- Alignment tolerance: ± 2 μm (adequate for expanded-mode edge couplers)
- Advantages: Higher throughput, lower cost (no active feedback required)

**Fiber Array Units (FAU):**
- Pre-aligned fiber ribbon bonded to silicon V-groove chip
- Pitch accuracy: ± 1 μm for 250 μm pitch arrays
- Applications: 12-fiber or 24-fiber MPO interfaces

---

### 3.3 Edge Coupling vs. Surface Coupling

| Method | Insertion Loss | Bandwidth | Alignment Tolerance | Testing |
|--------|----------------|-----------|---------------------|---------|
| Edge Coupling (SSC) | < 1.5 dB | > 100 nm | ± 0.5 μm | After dicing |
| Grating Coupler | 3-5 dB (standard), < 2 dB (advanced) | 30-80 nm | ± 1 μm | Wafer-scale |

**Recommendation:**
- Edge coupling: Production modules, low-loss requirement
- Grating couplers: Prototyping, wafer-scale testing, CPO with vertical fiber attachment

---

## 4. Thermal Management

### 4.1 Thermal Design Requirements

**Objectives:**
- Maintain photonic chip junction temperature: < 85°C (standard), < 70°C (premium)
- Minimize temperature gradients: < 5°C across die
- Enable thermal stability: ± 2°C for wavelength-critical applications (e.g., WDM)

**Thermal Resistance:**
- Die-to-case (θ_JC): < 5 °C/W for 3W dissipation
- Case-to-ambient (θ_CA): Depends on airflow, typically 10-20 °C/W with forced air

---

### 4.2 Cooling Techniques

**Passive Cooling:**
- Heat spreader (copper or aluminum) attached to package lid
- Thermal interface material (TIM): > 3 W/m·K thermal conductivity
- Heat sink fins: For natural or forced convection

**Active Cooling:**
- Fans: For modules > 10W power dissipation
- Thermoelectric coolers (TECs): For wavelength stabilization (±0.1°C control)
- Liquid cooling (micro-channels): For CPO with > 20W photonics power

**CPO-Specific:**
- Separate thermal domains for ASIC and photonics (isolate laser heat from ASIC)
- Thermal shunt structures to direct heat away from temperature-sensitive components

---

### 4.3 Temperature Sensing and Control

**On-Chip Temperature Sensors:**
- Type: PN junction diode, resistive temperature detector (RTD), or bandgap sensor
- Accuracy: ± 3°C absolute, ± 1°C relative
- Placement: Near lasers, modulators, and high-power components

**Feedback Control:**
- TEC controller with PID loop for laser temperature stabilization
- Heater power adjustment for thermal tuning of filters and modulators
- Adaptive power management to reduce dissipation at high ambient temperatures

---

## 5. Electrical Integration

### 5.1 Wire Bonding

**Applications:**
- Low-speed control signals (I2C, SPI)
- DC bias lines for modulators and photodetectors
- Power supply distribution

**Requirements:**
- Wire diameter: 25 μm (gold or aluminum)
- Loop height: < 300 μm to fit in low-profile packages
- Inductance: < 1 nH per bond (critical for high-speed signals)

**Limitations:**
- Not suitable for > 25 Gbps signals (excessive parasitic inductance)

---

### 5.2 Flip-Chip Bonding

**Applications:**
- High-speed modulator drivers (> 50 Gbps)
- Photodetector outputs (> 50 Gbps)
- Dense electrical I/O (> 100 connections)

**Bump Specifications:**
- Bump pitch: 100-150 μm
- Bump type: Solder (SnAg, AuSn), copper pillar, or hybrid
- Underfill: Required for mechanical stability and thermal cycling reliability

**Advantages:**
- Low parasitic inductance (< 100 pH per bump)
- High density (> 1000 I/O possible)
- Better thermal conduction to substrate

---

### 5.3 Through-Silicon Vias (TSV)

**For 3D Integration:**
- Via diameter: 5-20 μm
- Via pitch: 20-50 μm
- Aspect ratio: < 10:1
- Electrical resistance: < 50 mΩ per via

**Applications:**
- Backside power delivery (reduce IR drop)
- Vertical signal routing (multi-die stacks)
- Optical TSV (future): Vertical fiber coupling through silicon substrate

---

## 6. Co-Packaged Optics (CPO) Detailed Specifications

### 6.1 Integration Architectures

**Near-Package Optics:**
- Optical engine positioned adjacent to ASIC on PCB
- Electrical trace length: 1-3 cm
- SerDes data rate: 56 Gbps PAM4 (28 GBaud)

**On-Package Optics:**
- Photonic die on same package substrate as ASIC
- Electrical trace length: < 5 mm
- SerDes data rate: 112 Gbps PAM4 (56 GBaud) or higher

**In-Package Optics:**
- Photonic chiplet in same multi-die package as ASIC, using silicon interposer
- Electrical trace length: < 1 mm
- Direct optical coupling between dies (no fibers)

---

### 6.2 Optical I/O Configurations

**Edge-Attach Fiber Arrays:**
- Fiber ribbon with 12 or 24 fibers attached to package edge
- Connector type: MPO, CS, or custom
- Fiber routing: 90° bend within package using low-bend-radius fiber (< 5 mm radius)

**Top-Attach via Grating Couplers:**
- Fiber array positioned above photonic die through opening in package lid
- Connector integrated into lid assembly
- Advantage: Easier fiber access for rework/repair

**Waveguide Substrate Routing:**
- Optical waveguides in glass interposer route light to package edge
- Photonic die couples to waveguide via edge coupler or grating
- Enables arbitrary optical I/O placement

---

### 6.3 Power Budget for CPO

**Target Power Savings vs. Pluggable:**
- SerDes power reduction: 40-50% (due to shorter electrical reach)
- Removal of module DSP: ~20% of pluggable module power
- Total system power reduction: 30-45%

**Example (800G CPO):**
- ASIC-side SerDes: 6W (8 lanes × 0.75 W/lane)
- Photonics (lasers, modulators, detectors, drivers): 8W
- Control and monitoring: 1W
- **Total: 15W** (vs. 22W for pluggable solution)

---

### 6.4 Thermal Isolation for CPO

**Challenge:**
- ASIC generates 300-500W heat, while photonics requires stable temperature

**Solutions:**
- Separate heat sink for photonics (thermally isolated from ASIC)
- TEC for laser temperature control (adds 5-10W power, but critical for wavelength stability)
- Thermal buffer (low thermal conductivity material) between ASIC and photonics regions

---

## 7. Mechanical and Environmental Protection

### 7.1 Hermeticity

**Hermetic Sealing:**
- Required for long-term reliability (> 10 years) in uncontrolled environments
- Seal methods: Laser welding, glass sealing, or epoxy sealing with desiccant
- Leak rate: < 1×10⁻⁸ atm·cc/s (helium leak test)

**Non-Hermetic:**
- Adequate for data center environments (controlled temperature/humidity)
- Conformal coating for moisture resistance
- Cost advantage: 30-50% lower than hermetic

---

### 7.2 Mechanical Shock and Vibration

**Requirements (per Telcordia GR-63-CORE):**
- Shock: Survive 50G half-sine, 1 ms duration
- Vibration: 10-2000 Hz, 1G amplitude, 1 hour per axis
- Drop test: 1 meter onto concrete surface (packaged module)

**Design Considerations:**
- Compliant adhesive for die attach (reduces stress)
- Fiber strain relief boots (prevent fiber breakage at connector interface)
- Underfill for flip-chip to redistribute stress

---

## 8. Testing and Quality Assurance

### 8.1 Incoming Inspection

**Wafer-Level:**
- Visual inspection for defects (cracks, contamination)
- Optical parametric testing via probe station
- Yield mapping (identify known-good die)

**Die-Level (After Dicing):**
- Edge quality inspection (no chipping for edge couplers)
- Dimensional verification (thickness, facet angle)

---

### 8.2 Assembly Process Control

**Fiber Attachment:**
- Real-time monitoring of coupling efficiency during active alignment
- Epoxy cure verification (UV dose or thermal profile)
- Post-cure insertion loss verification (< 0.5 dB shift from pre-cure)

**Wire Bonding / Flip-Chip:**
- Pull test for wire bonds (> 5 grams force)
- X-ray inspection for flip-chip bump integrity
- Electrical continuity testing

---

### 8.3 Final Module Testing

**Optical Tests:**
- Optical power output (all channels)
- Receiver sensitivity
- Eye diagram quality (extinction ratio, jitter, eye opening)
- BER testing (< 10⁻¹² with FEC)

**Environmental Screening:**
- Temperature cycling: -5°C to 70°C, 5 cycles
- Burn-in: 48 hours at 70°C operating
- Final test at room temperature (verify no degradation)

---

## 9. Qualification and Reliability

### 9.1 Design Qualification Tests

**Performed Once per Design:**
- Full environmental testing (temperature, humidity, shock, vibration)
- Long-term accelerated life testing (HTOL: 2000 hours at 85°C)
- Interoperability testing with multiple host systems

---

### 9.2 Production Quality Metrics

**Targets:**
- First-pass yield: > 95% (modules passing final test without rework)
- Field return rate: < 0.1% per year
- MTBF: > 1,000,000 hours

---

**Published by:**
World Certification Industry Association (WIA)
弘益人間 (Hongik Ingan) - Benefit All Humanity

© 2025 WIA. All rights reserved.
