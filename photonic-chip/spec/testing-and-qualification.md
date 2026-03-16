# WIA-SEMI-006: Testing and Qualification Procedures

**Version:** 1.0
**Date:** 2025-01-15
**Editor:** World Certification Industry Association (WIA)

---

## 1. Overview

This specification establishes comprehensive testing and qualification procedures for photonic chips compliant with WIA-SEMI-006. It defines test methods, equipment requirements, pass/fail criteria, and reliability qualification protocols to ensure consistent quality and interoperability across vendors.

---

## 2. Test Categories

### 2.1 Hierarchy of Testing

1. **Wafer-Level Testing:** Performed on fabricated wafers before dicing
2. **Die-Level Testing:** Individual die characterization after dicing (for known-good-die selection)
3. **Module-Level Testing:** Packaged and fiber-attached modules
4. **System-Level Testing:** Installed in target platforms (interoperability, stress testing)
5. **Field Monitoring:** In-service performance tracking and failure analysis

---

## 3. Wafer-Level Optical Testing

### 3.1 Equipment Requirements

**Optical Probe Station:**
- Wafer chuck with vacuum hold-down and thermal control (-20°C to +85°C)
- Automated XYZ positioning (< 1 μm repeatability)
- Grating coupler or vertical fiber probes for optical access
- Vision system for die alignment and defect detection

**Light Sources:**
- Tunable laser: 1250-1650 nm range, < 100 kHz linewidth, > 10 mW output
- Broadband source: ASE or superluminescent LED for spectral measurements
- Fixed-wavelength lasers: 1310 nm and 1550 nm, calibrated power

**Detection:**
- Optical power meter: -50 to +10 dBm range, ± 0.1 dB accuracy
- Optical spectrum analyzer (OSA): 0.01 nm resolution, > 60 dB dynamic range
- Photodetector + oscilloscope: > 50 GHz bandwidth for high-speed characterization

---

### 3.2 Test Procedures

**Insertion Loss:**
1. Establish reference power (fiber-to-fiber coupling through calibration die)
2. Probe device under test (DUT) input and output
3. Measure transmitted power, calculate loss
4. **Pass Criteria:** Loss < 6 dB for modulators, < 4 dB for passive MUX/DEMUX

**Spectral Response:**
1. Sweep tunable laser across wavelength range (1510-1570 nm for C-band devices)
2. Record transmitted power vs. wavelength
3. Identify resonances, passbands, and filter shapes
4. **Pass Criteria:** Passband as specified in design (e.g., 0.8 nm for 100 GHz WDM channel)

**Crosstalk (for MUX/DEMUX):**
1. Inject light into one input channel
2. Measure power at all output channels
3. Calculate crosstalk ratio (wanted / unwanted channel power)
4. **Pass Criteria:** Adjacent channel < -25 dB, non-adjacent < -35 dB

---

### 3.3 Electrical-Optical (E-O) and Optical-Electrical (O-E) Testing

**Modulator E-O Bandwidth:**
- Equipment: Vector network analyzer (VNA) 67 GHz, photodetector + TIA
- Procedure: Apply RF sweep to modulator, detect modulated optical signal, measure S21
- **Pass Criteria:** -3 dB bandwidth > 50 GHz

**Photodetector O-E Bandwidth:**
- Equipment: Modulated optical source (from modulator or external), VNA
- Procedure: Illuminate photodetector, measure electrical output vs. frequency
- **Pass Criteria:** -3 dB bandwidth > 50 GHz, responsivity > 0.8 A/W

---

## 4. Die-Level Testing (Post-Dicing)

### 4.1 Visual and Dimensional Inspection

**Microscopy:**
- Check facet quality (no chipping, cracks)
- Verify waveguide termination (clean edge for edge coupling)
- Detect contamination or residue from dicing

**Metrology:**
- Die thickness: ± 5 μm tolerance
- Facet angle: 90° ± 1° (for straight edge couplers) or per design spec (angled facets)

---

### 4.2 Electrical Testing

**DC Characterization:**
- I-V curves for modulators (PN junctions), photodetectors
- Threshold voltage, leakage current, breakdown voltage
- **Pass Criteria:**
  - Modulator: Reverse leakage < 1 μA at -2V
  - Photodetector: Dark current < 100 nA at -1V, forward turn-on ~0.7V

**Capacitance:**
- C-V measurement for modulator and detector junctions
- **Pass Criteria:** Capacitance < 50 fF for > 50 GHz bandwidth devices

---

## 5. Module-Level Testing (Packaged Devices)

### 5.1 Optical Performance Tests

**Transmitter Tests:**
- **Optical Power:** Per-channel output power (typical: 0 to +5 dBm per lane)
- **Wavelength Accuracy:** ± 0.1 nm from ITU-T grid
- **SMSR (Side-Mode Suppression Ratio):** > 30 dB
- **Extinction Ratio (Static):** > 6 dB
- **Eye Diagram (100 Gbps NRZ or 100G PAM4):**
  - Eye opening > 80% (vertical and horizontal)
  - Jitter (RMS) < 1 ps
  - Overshoot < 20%

**Receiver Tests:**
- **Responsivity:** > 0.8 A/W at 1550 nm (measured with calibrated source)
- **Sensitivity:** Input power for BER = 10⁻¹² with FEC
  - Typical target: -10 to -15 dBm for 100G PAM4
- **Saturation Power:** Maximum input power before BER degradation
  - Typical: > -3 dBm

---

### 5.2 Bit Error Rate (BER) Testing

**Equipment:**
- Bit error rate tester (BERT) with pattern generator and error detector
- Variable optical attenuator (VOA) for sensitivity measurements
- Patterns: PRBS7, PRBS15, PRBS31 (Pseudo-Random Binary Sequence)

**Procedure:**
1. Set transmitter to nominal output power
2. Attenuate signal to receiver
3. Measure BER at various optical power levels
4. Generate BER vs. received power curve

**Pass Criteria:**
- BER < 10⁻¹² with FEC enabled at specified minimum receive power
- Error-free operation (< 1 error in 10¹⁵ bits) for 5+ minutes at nominal power

---

### 5.3 Link Performance Tests

**Transmitter and Dispersion Penalty (TDECQ):**
- Metric: Transmitter Dispersion Eye Closure Quaternary
- Measures transmitter quality accounting for chromatic dispersion
- **Pass Criteria:** TDECQ < 3.5 dB for 500m SMF links (400G/800G applications)

**Stressed Sensitivity:**
- Inject noise or interference (e.g., sinusoidal jitter) and measure sensitivity degradation
- **Pass Criteria:** < 1 dB sensitivity penalty with standard stress conditions

---

### 5.4 Multi-Channel and Crosstalk Tests

**For WDM Modules:**
1. Activate all channels simultaneously
2. Measure power and BER on each channel
3. Compare to single-channel performance

**Pass Criteria:**
- Power imbalance: < 2 dB across channels
- BER degradation: < 0.5 dB penalty due to crosstalk
- Spectral crosstalk: < -25 dB (measured with OSA)

---

## 6. Environmental and Reliability Testing

### 6.1 Temperature Testing

**Operating Temperature Range:**
- Standard: 0°C to 70°C
- Extended: -5°C to 85°C

**Test Procedure:**
1. Stabilize module at low temperature extreme (-5°C or 0°C)
2. Perform full optical and BER tests
3. Ramp to high temperature extreme (70°C or 85°C)
4. Perform full optical and BER tests
5. Return to room temperature (25°C), verify recovery

**Pass Criteria:**
- Optical power variation: < 1.5 dB across temperature range
- BER < 10⁻¹² at all temperatures
- Wavelength drift: < ± 0.5 nm (acceptable if within grid tolerance)

---

### 6.2 Temperature Cycling

**Profile:**
- Low temperature: -40°C, dwell 15 minutes
- High temperature: +85°C, dwell 15 minutes
- Ramp rate: 10-20°C per minute
- Number of cycles: 500 (qualification), 5 (production screening)

**Monitoring:**
- Optical power (in-situ if possible, or at intervals)
- Detect delamination, bond failures via visual inspection and SAM (Scanning Acoustic Microscopy)

**Pass Criteria:**
- No cracks, delamination, or bond failures
- Optical power degradation < 0.5 dB after cycling

---

### 6.3 High-Temperature Operating Life (HTOL)

**Conditions:**
- Temperature: 85°C (standard), 125°C (accelerated)
- Duration: 1000-2000 hours
- Bias: Modulators and lasers energized, typical operating current
- Monitoring: Sample devices periodically (every 168 hours) for optical power, threshold current, dark current

**Pass Criteria:**
- Optical power degradation: < 1 dB over 2000 hours
- Dark current increase: < 2× initial value
- Threshold current (lasers): < 20% increase
- Extrapolated lifetime: > 15 years at 40°C operating temperature (using Arrhenius model)

---

### 6.4 Humidity and Moisture Resistance

**Test (85/85):**
- Temperature: 85°C
- Relative humidity: 85%
- Duration: 1000 hours (non-operating)
- Postcondition: Bake at 85°C dry for 24 hours before testing

**Pass Criteria:**
- No corrosion, delamination, or moisture ingress
- Electrical and optical performance within initial specifications

---

### 6.5 Mechanical Shock and Vibration

**Shock Test (per Telcordia GR-63):**
- Amplitude: 50G peak
- Pulse shape: Half-sine
- Duration: 1 ms
- Directions: 6 directions (±X, ±Y, ±Z)

**Vibration Test:**
- Frequency range: 10-2000 Hz
- Amplitude: 1G
- Duration: 1 hour per axis (3 axes total)

**Pass Criteria:**
- No physical damage (cracks, detachment)
- Optical performance within specification (< 0.5 dB change)

---

## 7. Interoperability Testing

### 7.1 Multi-Vendor Interoperability

**Objective:**
- Verify that transceivers from different vendors can establish error-free links

**Test Setup:**
- Vendor A transmitter → Vendor B receiver (and vice versa)
- Use reference fibers with defined loss and dispersion
- Test across temperature range and with various cable lengths

**Pass Criteria:**
- BER < 10⁻¹² for all vendor combinations
- Link training and auto-negotiation successful (for protocols supporting it)

---

### 7.2 Host System Compatibility

**Platforms to Test:**
- Major switch ASICs (Broadcom Tomahawk, Cisco Silicon One, Marvell Teralynx)
- Network interface cards (NICs) from Intel, Nvidia/Mellanox, Broadcom
- Coherent DSP platforms (for coherent transceivers)

**Pass Criteria:**
- Successful link establishment (within 1 second)
- Stable operation for 24+ hours (no link flaps)
- Firmware and management interface compatibility (CMIS, etc.)

---

## 8. Compliance and Certification

### 8.1 WIA-SEMI-006 Certification Process

**Submission Requirements:**
- Complete test reports (wafer, die, module levels)
- Design documentation (schematics, layout, BOM)
- Reliability data (HTOL, temperature cycling, qualification test summary)
- Interoperability matrix (tested combinations)

**Review Process:**
- WIA technical committee reviews submission (4-6 weeks)
- May request additional testing or clarification
- Certification issued upon approval

**Certification Mark:**
- Modules passing certification may display WIA-SEMI-006 logo
- Certification valid for 3 years, renewable with updated test data

---

### 8.2 Ongoing Compliance Monitoring

**Field Performance Tracking:**
- Certified vendors must report field failure rates annually
- Failures exceeding 0.5% per year trigger review

**Periodic Re-Testing:**
- Random sampling of production units for compliance verification (1 in 1000 modules)
- WIA may conduct independent testing with purchased modules

---

## 9. Test Equipment Calibration

### 9.1 Calibration Requirements

**Optical Power Meters:**
- Calibration frequency: Annual
- Traceable to NIST or equivalent national standards body
- Calibration wavelengths: 1310 nm, 1550 nm minimum

**Optical Spectrum Analyzers:**
- Wavelength accuracy calibration: Semi-annual
- Power level calibration: Annual
- Wavelength reference: Acetylene gas cell or stabilized laser

**Electrical Test Equipment (VNA, Oscilloscopes):**
- Calibration frequency: Annual
- Performed by equipment manufacturer or certified calibration lab

---

## 10. Documentation and Record Keeping

### 10.1 Test Records

**Required Information:**
- Date and time of test
- Equipment used (model, serial number, calibration date)
- Environmental conditions (temperature, humidity)
- Test results (pass/fail, measured values)
- Operator ID and signature

**Retention:**
- Wafer-level data: 2 years
- Module test data: Lifetime of product + 5 years
- Qualification test data: Permanent retention

---

### 10.2 Failure Analysis Reports

**For Failed Units:**
- Description of failure mode (optical, electrical, mechanical)
- Root cause analysis (RCA)
- Corrective actions implemented
- Effectiveness verification (retest after corrective action)

**Sharing with WIA:**
- Critical failures (safety hazard, widespread field failures) must be reported to WIA within 30 days

---

**Published by:**
World Certification Industry Association (WIA)
弘益人間 (Hongik Ingan) - Benefit All Humanity

© 2025 WIA. All rights reserved.
