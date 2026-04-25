# WIA-SEMI-018 Wafer Specification v1.0

## Silicon Wafer Quality Standards for Advanced Semiconductor Manufacturing

**Document Number**: WIA-SEMI-018-SPEC-001
**Version**: 1.0
**Effective Date**: January 1, 2025
**Revision Date**: January 1, 2025

---

## 1. Scope

This specification defines the quality requirements for silicon wafers used in semiconductor device manufacturing, covering 300mm diameter wafers for advanced logic and memory applications at technology nodes from 180nm down to 2nm and beyond.

---

## 2. Referenced Standards

- SEMI M1-0320: Specifications for Polished Monocrystalline Silicon Wafers
- SEMI M59-0218: Terminology for Silicon Technology
- SEMI C1-0218: Specifications for Reagents
- ASTM F1530: Standard Test Method for Surface Roughness of Silicon Wafers
- WIA-SEMI-018: Semiconductor Material Standard (Parent Document)

---

## 3. Silicon Purity Requirements

### 3.1 General Purity Specification
**Minimum Purity**: 11-9s (99.999999999%)

**Maximum Total Impurity**: 10 ppb (parts per billion) atomic

### 3.2 Specific Element Limits

| Element | Symbol | Maximum Concentration (ppba) | Test Method |
|---------|--------|------------------------------|-------------|
| Boron | B | 0.3 | SIMS |
| Phosphorus | P | 0.2 | SIMS |
| Arsenic | As | 0.1 | SIMS |
| Carbon | C | 0.5 | SIMS/FTIR |
| Oxygen (CZ wafers) | O | 0.5-1.0 (controlled range) | FTIR |
| Oxygen (FZ wafers) | O | <0.01 | FTIR |
| Iron | Fe | 0.05 | SIMS/TXRF |
| Copper | Cu | 0.01 | SIMS/TXRF |
| Nickel | Ni | 0.01 | SIMS/TXRF |
| Chromium | Cr | 0.01 | SIMS/TXRF |
| Aluminum | Al | 0.1 | SIMS |
| Sodium | Na | 0.05 | SIMS |
| Potassium | K | 0.05 | SIMS |

**Test Methodology**:
- **SIMS** (Secondary Ion Mass Spectrometry): Primary method for trace element detection
  - Detection limit: 10¹⁴-10¹⁵ atoms/cm³
  - Sampling: Minimum 1 wafer per production lot
  - Depth profiling: 50-100 nm minimum

- **FTIR** (Fourier Transform Infrared Spectroscopy): For oxygen and carbon
  - Resolution: 4 cm⁻¹
  - Sampling: 5 points per wafer

- **TXRF** (Total Reflection X-Ray Fluorescence): Surface contamination
  - Detection limit: 10⁹-10¹⁰ atoms/cm²
  - Sampling: 5 wafers per lot

---

## 4. Dimensional Specifications (300mm Wafers)

### 4.1 Diameter
**Specification**: 300.0 ± 0.2 mm

**Test Method**: Optical comparator or laser measurement
**Sampling**: 100% of wafers
**Measurement Points**: Minimum 4 points around circumference

### 4.2 Thickness
**Specification**: 775 ± 10 µm

**Test Method**: Capacitance gauge or optical interference
**Sampling**: 100% of wafers
**Measurement Points**: Minimum 25 points across wafer surface

**Thickness Mapping**: Required for qualification lots
- Grid: 5mm spacing
- Report: Full wafer thickness map with statistical analysis

### 4.3 Total Thickness Variation (TTV)
**Specification**: <2.0 µm

**Calculation**: TTV = (Maximum thickness - Minimum thickness) across entire wafer
**Test Method**: Capacitance gauge thickness mapping
**Sampling**: 25 wafers per lot minimum

### 4.4 Bow
**Specification**: <40 µm

**Definition**: Maximum deviation of median surface from reference plane (3-point mount)
**Test Method**: Capacitance gauge or laser scanner
**Sampling**: 25 wafers per lot

### 4.5 Warp
**Specification**: <50 µm

**Definition**: Maximum range of wafer surface from reference plane
**Test Method**: Capacitance gauge or laser scanner
**Sampling**: 25 wafers per lot

### 4.6 Site Flatness (SFQR)
**Specification**: <100 nm (over 26mm × 33mm site)

**Definition**: Site Flatness Focal plane QR - flatness over lithography exposure field
**Test Method**: Advanced capacitance gauge
**Application**: Critical for advanced lithography (7nm and below nodes)
**Sampling**: Qualification lots and monthly monitoring

### 4.7 Nanotopography
**Specification**: <50 nm PV (peak-to-valley over 2mm × 2mm site)

**Test Method**: Advanced optical or capacitance measurement
**Application**: 5nm and below nodes
**Sampling**: Qualification lots

### 4.8 Edge Profile
**Profile Type**: Rounded edge (standard)
**Edge Exclusion Zone**: 2.0 mm from edge

**Specifications**:
- Edge radius: 0.2-0.4 mm
- Edge roll-off: <5 µm over 1mm from edge

### 4.9 Notch (Orientation Marker)
**Depth**: 0.8 ± 0.1 mm
**Width**: 3.0 ± 0.3 mm at surface
**Angle**: 90° ± 5°
**Location**: Primary flat position (if notch only wafer)

---

## 5. Crystal Properties

### 5.1 Crystal Orientation
**Standard Orientation**: <100> ± 0.5°

**Alternative Orientations** (by customer specification):
- <111> ± 0.5°
- <110> ± 0.5°

**Test Method**: X-ray diffraction (XRD)
**Sampling**: 1 wafer per lot

### 5.2 Crystal Type
**Standard**: Czochralski (CZ) grown, p-type or n-type as specified

**Alternative**: Float Zone (FZ) for high-resistivity or ultra-low oxygen applications

### 5.3 Resistivity
**Specification**: As ordered (typical ranges):
- 1-10 Ω·cm (lightly doped logic)
- 10-20 Ω·cm (lightly doped memory)
- 0.001-0.01 Ω·cm (heavily doped, substrate)
- >1000 Ω·cm (high-resistivity, RF/power devices)

**Uniformity**: ± 10% across wafer (1σ)

**Test Method**: Four-point probe (ASTM F84)
**Sampling**: 25 wafers per lot (9 or 25 points per wafer depending on requirement)

### 5.4 Minority Carrier Lifetime
**Specification**: >1 ms (for device-grade wafers)

**Test Method**: µPCD (microwave photoconductivity decay) or SPV (surface photovoltage)
**Application**: Critical for power devices and some analog applications

---

## 6. Surface Quality

### 6.1 Surface Roughness
**Specification**: Ra < 0.2 nm

**Test Method**: Atomic Force Microscopy (AFM)
- Scan area: 10 µm × 10 µm minimum
- Scan resolution: <10 nm per pixel
- Mode: Tapping mode (non-contact preferred)

**Sampling**: 5 wafers per qualification lot, 1 wafer per production lot (monthly)

**Measurement Points**: Center and 4 edge points (10-15mm from edge)

### 6.2 Microroughness (Haze)
**Specification**: <0.10 ppm (parts per million) equivalent surface area

**Test Method**: Laser scattering surface scanner
**Sampling**: 100% of wafers (inline during manufacturing)

### 6.3 Particle Defects
**Specification**: <0.08 defects/cm² (>0.09 µm particles)

**Breakdown by Defect Size**:
- >0.09 µm: <0.08 /cm²
- >0.12 µm: <0.05 /cm²
- >0.16 µm: <0.03 /cm²
- >0.20 µm: <0.02 /cm²

**Test Method**: Automated laser scattering particle scanner
**Sampling**: 100% of wafers

**Acceptance Criteria**:
- Production wafers: Meet specification
- Test/monitor wafers: <0.15 defects/cm² acceptable
- Reject: >0.12 defects/cm² (20% margin above spec)

### 6.4 Localized Light Scatterers (LLS)
**Specification**: <0.05 LLS/cm² (detectable by advanced scanners)

**Definition**: Sub-surface defects or crystal-originated particles (COPs) 10-100nm in size

**Test Method**: Advanced laser scattering with multiple wavelengths and angles
**Application**: 7nm and below nodes
**Sampling**: Qualification lots and periodic monitoring (quarterly)

### 6.5 Surface Pits and Scratches
**Pits**:
- Depth: <10 nm
- Density: <5 pits per wafer (>10 µm diameter)

**Scratches**:
- Depth: <10 nm
- Length: <50 µm
- Density: <2 scratches per wafer (length >1mm)

**Test Method**: Optical inspection and AFM spot checks

### 6.6 Metallic Surface Contamination
**Specification**: <1 × 10¹⁰ atoms/cm² for each element

**Critical Elements** (lower limits):
- Iron (Fe): <5 × 10⁹ atoms/cm²
- Copper (Cu): <5 × 10⁹ atoms/cm²
- Nickel (Ni): <3 × 10⁹ atoms/cm²

**Test Method**: TXRF or VPD-ICPMS (Vapor Phase Decomposition ICP-MS)
**Sampling**: 5 wafers per lot (qualification), monthly monitoring (production)

---

## 7. Packaging and Handling

### 7.1 Wafer Cassette
**Type**: 300mm FOSB (Front Opening Shipping Box) compliant

**Cassette Material**: Antistatic polycarbonate or PBT (polybutylene terephthalate)
**Capacity**: 25 wafers per cassette
**Wafer Spacing**: 10mm pitch
**Cleanliness**: Class 1 (ISO 3) or better

### 7.2 Secondary Packaging
**Inner Bag**: Antistatic, heat-sealed with desiccant
- Material: Polyethylene with ESD protection
- Desiccant: Silica gel, sufficient for 30-day protection

**Outer Container**: FOSB-compliant rigid container
- Shock protection: >4G shock resistance
- Moisture barrier: Prevents humidity ingress
- Sealable: Tamper-evident seal

### 7.3 Labeling
**Required Information on Cassette**:
- Wafer lot number
- Wafer quantity
- Crystal orientation (<100>, etc.)
- Doping type (p or n)
- Resistivity range
- Supplier name and site
- Manufacturing date
- Barcode (Code 128 or similar)

**Wafer Identification**:
- Laser-marked ID on wafer edge (300mm wafers, optional but recommended)
- ID includes: Lot number, sequential number, date code

### 7.4 Handling Guidelines
**Environment**: Cleanroom Class 100 (ISO 5) or better during unpacking
**Gloves**: Powder-free nitrile or vinyl
**Tools**: Wafer tweezers or vacuum wands (avoid metal contact with front surface)
**Storage**: Horizontal position in original cassette, avoid stacking
**Cleanroom Protocol**: Full cleanroom garments, minimal particle generation

---

## 8. Quality Control and Testing

### 8.1 100% Inspection Parameters
- Diameter
- Thickness
- Particle defects (laser scan)
- Visual inspection (automated optical)

### 8.2 Sampling Inspection Parameters
**Per Lot** (25 wafer minimum sample):
- TTV
- Bow
- Warp
- Resistivity

**Per Lot** (5 wafer sample):
- Surface roughness (AFM)
- Metallic contamination (TXRF)

**Per Lot** (1 wafer):
- Purity analysis (SIMS)
- Crystal orientation (XRD)

### 8.3 Periodic Special Testing
**Monthly** (or per customer agreement):
- Site flatness (SFQR)
- Nanotopography
- LLS detection
- Minority carrier lifetime

**Quarterly**:
- Full material characterization (all parameters)
- Process qualification wafers (customer-specific)

### 8.4 Certificate of Analysis (CoA)
**Required Data** (shipped with each lot):
- Lot number and quantity
- All 100% inspection results (statistical summary)
- Sampling inspection results (average, range, Cpk)
- SIMS purity data (with spectrum)
- Resistivity map (if applicable)
- Any out-of-spec conditions or deviations
- Traceability to raw material (ingot ID)

**Format**: Electronic (PDF or XML) and physical copy

---

## 9. Acceptance Criteria and Disposition

### 9.1 Acceptance
**Criteria**: All parameters within specification limits as defined in Sections 3-6

**Sampling Plan**: AQL 0.15% (ISO 2859-1 Level II, Normal Inspection)
- Lot size: 500 wafers
- Sample size: 80 wafers
- Accept: 1 defect
- Reject: 2 defects

**Note**: 100% inspection parameters (particle defects) use individual wafer acceptance

### 9.2 Non-Conformance Handling
**Minor Deviations** (within 10% of specification):
- Conditional acceptance with documentation
- Root cause analysis required
- Preventive action plan from supplier

**Major Deviations** (>10% of specification or safety concern):
- Lot rejection
- Return to supplier for credit or rework
- Corrective action required before next shipment

### 9.3 Wafer Grading
**Prime Grade**: All specs met, suitable for production
**Test Grade**: Minor deviations, suitable for process development only
**Mechanical Grade**: Dimensional specs met, suitable for mechanical/tool qualification
**Reclaim Grade**: Used wafers suitable for strip and repolish

---

## 10. Supplier Qualification Requirements

### 10.1 Initial Qualification
**Duration**: 3-6 months
**Sample Size**: Minimum 3 lots (75 wafers per lot)
**Testing**: Full characterization (all parameters in Section 8)
**Process Capability**: Cpk ≥ 1.33 for all critical parameters

### 10.2 Ongoing Monitoring
**Frequency**: Monthly scorecards
**Metrics**:
- On-time delivery: >98%
- Quality (defect rate): <100 ppm
- CoA accuracy: 100%
- Responsiveness: <24 hours for technical issues

### 10.3 Re-qualification
**Triggers**:
- Change in manufacturing process or facility
- Quality issues (>2 rejected lots in 6 months)
- Merger/acquisition or ownership change
- New product introduction

**Frequency**: Minimum every 3 years for active suppliers

---

## 11. Environmental and Safety Considerations

### 11.1 Hazardous Materials
**Silicon Wafers**: Non-hazardous under normal handling conditions

**Precautions**:
- Sharp edges: Handle with care to avoid cuts
- Broken wafers: Generate silicon dust (potential respiratory irritant), clean up with HEPA vacuum
- Chemical residues: Some wafers may have trace chemicals from manufacturing, handle with gloves

### 11.2 Disposal
**Waste Classification**: Generally non-hazardous industrial waste

**Recycling**: Encourage reclaim and recycling programs
- Test wafers: Suitable for multiple strip/repolish cycles
- Broken/scrap wafers: Recycling for polysilicon feedstock

### 11.3 Sustainability
**Supplier Commitments** (encouraged):
- Energy efficiency in crystal growth and wafer processing
- Water recycling (target: >50% reuse)
- Packaging material reduction and recyclability
- Carbon footprint reporting and reduction targets

---

## 12. Revision History

| Version | Date | Changes | Approved By |
|---------|------|---------|-------------|
| 1.0 | 2025-01-01 | Initial release | WIA Standards Committee |

---

## 13. Appendices

### Appendix A: Test Method Details
(See referenced SEMI and ASTM standards for full protocols)

### Appendix B: Wafer Handling Best Practices
- Always handle in cleanroom environment (Class 100 or better)
- Use proper tools (vacuum wands, ESD-safe tweezers)
- Store in original containers, sealed with desiccant
- Avoid temperature shocks (>5°C/hour change)
- FIFO inventory management (use oldest stock first)

### Appendix C: Troubleshooting Guide
**High Particle Count**:
- Check cassette cleanliness
- Review cleanroom protocol compliance
- Inspect packaging integrity
- Audit supplier cleaning process

**Thickness Variation Issues**:
- Review grinding and polishing parameters
- Check CMP slurry and pad condition
- Verify tool calibration

**Purity Concerns**:
- Audit polysilicon source material
- Review crucible quality (CZ method)
- Check chemical purity (etchants, cleaning solutions)

---

**Contact Information**:
WIA Semiconductor Standards Division
Email: semiconductor@wia-standards.org
Web: https://wia-standards.org/semi-018

**Document Control**:
Filename: WIA-SEMI-018-SPEC-001-Wafer-v1.0.md
SHA-256: [To be calculated upon final release]

---

© 2025 SmileStory Inc. / World Certification Industry Association (WIA)
弘益人間 (홍익인간) · Benefit All Humanity
