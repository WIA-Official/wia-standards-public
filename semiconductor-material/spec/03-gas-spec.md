# WIA-SEMI-018 Specialty Gas Specification v1.0

## Ultra-High Purity Gas Standards

**Document Number**: WIA-SEMI-018-SPEC-003
**Version**: 1.0
**Effective Date**: January 1, 2025

---

## 1. Scope

This specification defines purity and handling requirements for specialty gases used in semiconductor manufacturing processes including doping, etching, deposition, and carrier applications.

---

## 2. General Purity Requirements

### 2.1 Minimum Purity Grades

| Gas Category | Minimum Purity | Application |
|--------------|---------------|-------------|
| Dopant Gases | 5N (99.999%) | Ion implantation, diffusion |
| Etch Gases | 4.5N-5N (99.995-99.999%) | Plasma etching, cleaning |
| CVD Precursors | 6N (99.9999%) | Film deposition |
| Carrier Gases | 6N-7N (99.9999-99.99999%) | Critical processes |

---

## 3. Specific Gas Specifications

### 3.1 Phosphine (PH₃)

**Purity**: 99.9999% (6N) minimum
**Typical Concentration**: 1-10% in H₂ or N₂

**Maximum Impurities**:
- Arsine (AsH₃): <1 ppm
- Moisture (H₂O): <1 ppm
- Oxygen (O₂): <0.5 ppm
- Nitrogen (N₂): Balance (for diluted)
- Particles: <1/ft³ (>0.3 µm)

**Safety Classification**:
- DOT: 2.3 (Poison Gas), 2.1 (Flammable Gas)
- UN Number: UN2199
- TLV-TWA: 0.3 ppm

---

### 3.2 Diborane (B₂H₆)

**Purity**: 99.999% (5N) typical, 99.9999% (6N) for advanced nodes
**Concentration**: 1-10% in H₂ or N₂

**Maximum Impurities**:
- Silane (SiH₄): <50 ppm
- Phosphine (PH₃): <10 ppm (critical - opposite doping)
- Moisture: <1 ppm
- Oxygen: <0.5 ppm

**Safety**:
- UN Number: UN1911
- TLV-TWA: 0.1 ppm
- Pyrophoric: Yes

---

### 3.3 Silane (SiH₄)

**Purity**: 99.9999% (6N) for advanced applications
**Form**: 100% pure or 2-10% in H₂/N₂

**Maximum Impurities**:
- Diborane: <0.1 ppm
- Phosphine: <0.1 ppm
- Arsine: <0.01 ppm
- CO/CO₂: <0.5 ppm
- Moisture: <0.5 ppm

**Safety**:
- UN Number: UN2203
- Pyrophoric: Spontaneously combustible in air
- Flammability range: 1.4-96% in air

---

### 3.4 Nitrogen Trifluoride (NF₃)

**Purity**: 99.99% (4N) to 99.998% (4.8N)

**Maximum Impurities**:
- CF₄: <100 ppm
- O₂: <50 ppm
- N₂: <500 ppm
- Moisture: <5 ppm

**Applications**: CVD chamber cleaning, silicon etch
**GWP**: 16,100 (CO₂ equivalent over 100 years)
**Abatement**: Required (>90% destruction efficiency)

---

### 3.5 Carrier Gases

#### Nitrogen (N₂)
**Purity**: 99.9999% (6N) to 99.99999% (7N)

**Maximum Impurities**:
- Oxygen: <1 ppm (6N), <0.1 ppm (7N)
- Moisture: <1 ppm
- Hydrocarbons: <0.1 ppm
- CO/CO₂: <0.5 ppm

#### Hydrogen (H₂)
**Purity**: 99.9999% (6N) to 99.99999% (7N)

**Maximum Impurities**:
- Oxygen: <0.1 ppm
- Moisture: <0.5 ppm
- CO/CO₂: <0.1 ppm
- Hydrocarbons: <0.1 ppm

#### Argon (Ar)
**Purity**: 99.9999% (6N) to 99.99999% (7N)

**Maximum Impurities**:
- Oxygen: <0.5 ppm
- Nitrogen: <5 ppm
- Moisture: <1 ppm

---

## 4. Cylinder and Packaging Requirements

### 4.1 Cylinder Specifications
- **Material**: DOT or ISO certified
- **Pressure Rating**: Appropriate for gas (typically 2400-3000 psig)
- **Valve**: CGA connection standard for specific gas
- **Testing**: Hydrostatic test every 5-10 years

### 4.2 Labeling
Required information:
- Gas name and chemical formula
- Purity grade
- UN number and hazard class
- Lot number and fill date
- Supplier name and emergency contact
- Net weight or volume
- Barcode for tracking

### 4.3 Protective Packaging
- Valve protection cap
- Cylinder secured during transport
- Proper hazard placarding on vehicle

---

## 5. Quality Control and Testing

### 5.1 Supplier Testing
**Frequency**: Every production batch

**Analysis Methods**:
- Gas Chromatography-Mass Spectrometry (GC-MS)
- Fourier Transform Infrared (FTIR)
- Residual Gas Analyzer (RGA)
- Moisture analyzer (tunable diode laser)

**Certificate of Analysis** (CoA):
- Batch number
- Fill date
- Purity (%age)
- All impurity levels (ppm)
- Test methods used
- Expiration date (if applicable)

### 5.2 Fab Incoming Inspection
**100% Inspection**:
- Cylinder exterior condition
- Valve integrity
- Label accuracy
- CoA review

**Sampling Inspection**:
- Analytical verification: 5-10% of cylinders
- Use inline RGA or gas purity monitor

### 5.3 Point-of-Use Verification
**Continuous Monitoring**:
- Gas purity monitors on critical lines
- Alarm setpoints: <specification limit

**Purifier Performance**:
- Inlet vs. outlet purity measurement
- Replace when breakthrough detected

---

## 6. Gas Delivery System Requirements

### 6.1 Materials of Construction
- Piping: Electropolished 316L stainless steel
- Valves: Metal-sealed, bellows-sealed preferred
- Regulators: Two-stage, high-purity compatible
- Fittings: VCR or ConFlat connections

### 6.2 System Cleanliness
**Preparation**:
- Chemical cleaning (remove oils, organics)
- High-purity water rinse
- Dry with high-purity nitrogen
- Vacuum bake-out (300-400°C if compatible)

**Leak Testing**:
- Helium mass spectrometer leak detector
- Specification: <1 × 10⁻⁹ atm·cc/sec

### 6.3 Purification
**Point-of-Use Purifiers** (recommended for critical gases):
- Getter-based (multi-element removal)
- Particle filters: 0.003 µm
- Target: Upgrade gas to 8-9N purity

---

## 7. Safety Requirements

### 7.1 Gas Cabinet Design
- Ventilated enclosure: 100-150 air changes/hour
- Seismic restraint: Zone-appropriate
- Gas detection: Electrochemical or other suitable sensor
- Emergency shut-off: Manual and automatic
- Fire suppression: For pyrophoric gases

### 7.2 Monitoring and Alarms
**Fixed-Point Detectors**:
- Location: Gas cabinet, exhaust ducts
- Alarm levels:
  - Low: 50% TLV-TWA (investigate)
  - High: TLV-TWA (evacuate)
  - IDLH: Facility-wide alarm

**Calibration**: Monthly for electrochemical sensors

### 7.3 Emergency Response
**Leak Response Procedure**:
1. Alarm acknowledgment
2. Evacuate affected area
3. Shut off gas supply
4. Boost ventilation
5. Measure concentration with portable monitor
6. Emergency team response (SCBA required)
7. Neutralize/scrub released gas

**Scrubber Systems**:
- Wet scrubbers: Acid gases (HCl, Cl₂, HF)
- Burn boxes: Pyrophorics (SiH₄, PH₃, B₂H₆)
- Destruction efficiency: >99%

---

## 8. Environmental Compliance

### 8.1 Greenhouse Gas Management
**PFCs** (SF₆, CF₄, C₂F₆, NF₃):
- Abatement: >90% destruction efficiency required
- Reporting: Kyoto Protocol compliance
- Alternative chemistries: Encouraged where feasible

### 8.2 Waste Gas Treatment
- Toxic gas scrubbing: >99% removal
- Effluent monitoring: Continuous or periodic sampling
- Discharge limits: Local regulatory compliance

---

## 9. Storage and Shelf Life

### 9.1 Storage Conditions
- Location: Dedicated gas storage area, segregated by hazard class
- Temperature: Ambient (protect from extreme heat/cold)
- Ventilation: Adequate to prevent accumulation
- Security: Restricted access, inventory control

### 9.2 Shelf Life
Most gases: Indefinite if cylinder integrity maintained

**Exceptions**:
- Diborane (B₂H₆): 12-18 months (decomposes over time)
- Germane (GeH₄): 12 months
- Some metal-organic precursors: As specified by supplier

**Inventory Management**: FIFO (first-in-first-out)

---

## 10. Acceptance Criteria

### 10.1 Acceptance
- All purity specifications met (per CoA)
- Cylinder and valve in good condition
- Documentation complete and accurate
- Sample verification (if performed) within spec

### 10.2 Rejection Criteria
- Purity below specification
- Cylinder damage or valve leak
- Missing or inaccurate CoA
- Expired (if shelf life limited)

**Disposition**: Return to supplier for credit or replacement

---

## 11. Supplier Qualification

### 11.1 Initial Qualification
**Requirements**:
- ISO 9001 certified or equivalent QMS
- Safety audit (emergency response, training)
- Capacity verification
- Sample analysis (3 batches minimum)

**Duration**: 2-4 months

### 11.2 Ongoing Monitoring
**Metrics**:
- Quality: <0.1% reject rate
- Delivery: >98% on-time
- Documentation: 100% accurate
- Responsiveness: <24 hour for technical issues

**Quarterly Business Reviews**: Recommended for critical gases

---

## 12. Revision History

| Version | Date | Changes | Approved By |
|---------|------|---------|-------------|
| 1.0 | 2025-01-01 | Initial release | WIA Standards Committee |

---

**Contact**:
WIA Semiconductor Standards Division
Email: semiconductor@wia-standards.org
Web: https://wia-standards.org/semi-018

© 2025 WIA | 弘益人間 (Benefit All Humanity)
