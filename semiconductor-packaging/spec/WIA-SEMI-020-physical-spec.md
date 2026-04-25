# WIA-SEMI-020: Physical Layer Specification for Semiconductor Packaging

## Document Information

- **Standard**: WIA-SEMI-020
- **Version**: 1.0
- **Date**: 2025-01-01
- **Status**: Published
- **Category**: Semiconductor Packaging - Physical Layer

## 1. Scope

This specification defines the physical layer requirements for advanced semiconductor packaging technologies including 2.5D, 3D, fan-out, and chiplet-based integration. It covers materials, dimensions, interconnections, and physical properties required for compliant implementations.

## 2. Normative References

- JEDEC JEP95: 2.5D Silicon Interposer Design Guide
- JEDEC JESD235: High Bandwidth Memory (HBM) DRAM
- IPC-6012: Qualification and Performance Specification for Rigid Printed Boards
- UCIe Specification v1.0: Universal Chiplet Interconnect Express
- JEDEC JEP154: Survey of TSV Application Spaces

## 3. Terms and Definitions

### 3.1 Advanced Packaging
Integration of multiple dies in a single package using 2.5D, 3D, or fan-out technologies to achieve performance, density, or cost objectives beyond traditional packaging.

### 3.2 Through-Silicon Via (TSV)
Vertical electrical connection passing through a silicon wafer or die, providing connectivity between the front and back sides.

### 3.3 Microbump
Fine-pitch solder bump interconnection, typically 20-55 μm pitch, used for die-to-die or die-to-interposer connections.

### 3.4 Redistribution Layer (RDL)
Metal routing layer(s) that redistribute I/O connections from fine-pitch die pads to coarser-pitch external connections.

### 3.5 Interposer
Intermediate substrate providing high-density interconnections between multiple dies and the package substrate in 2.5D configurations.

### 3.6 Chiplet
Modular integrated circuit die designed for heterogeneous integration with other chiplets via standardized interfaces.

## 4. Physical Layer Requirements

### 4.1 2.5D Packaging Physical Specifications

#### 4.1.1 Silicon Interposer

**Dimensions**:
- Thickness: 50-100 μm (after thinning)
- Maximum lateral dimension: 100 mm × 100 mm
- Thickness tolerance: ±5 μm
- Bow and warp: <100 μm over full interposer area

**Material Properties**:
- Material: High-resistivity silicon (>1000 Ω·cm) or silicon with insulating layer
- Crystal orientation: <100> preferred
- Thermal conductivity: ≥130 W/m·K

**TSV Specifications**:
- Diameter: 5-20 μm
- Depth: Equal to interposer thickness
- Pitch: ≥10 μm (center-to-center)
- Aspect ratio: 5:1 to 20:1
- TSV density: Up to 10,000 per mm²
- Resistance: <50 mΩ per TSV
- Capacitance: <10 fF per TSV
- Isolation: >100 GΩ at 125°C

**RDL Specifications**:
- Metal layers: 2-10 layers
- Metal: Copper (preferred) or aluminum
- Line width: 0.4-2 μm
- Line spacing: 0.4-2 μm
- Metal thickness: 1-5 μm per layer
- Via diameter: 0.5-3 μm
- Dielectric: Silicon dioxide, low-k materials
- Dielectric thickness: 0.5-3 μm between metal layers

#### 4.1.2 Microbump Interconnections

**Bump Specifications**:
- Pitch: 40-55 μm standard; down to 20 μm for advanced implementations
- Bump height: 15-25 μm
- Bump diameter: 25-40 μm
- Coplanarity: ±5 μm within a die
- Stand-off height after bonding: 10-15 μm

**Materials**:
- Copper pillar with SnAg solder cap (preferred)
- Pure tin or SAC alloy solder
- UBM stack: Ti/Ni(V)/Cu or similar

**Reliability**:
- Shear strength: ≥20 MPa
- Resistance: <10 mΩ per bump
- Electromigration lifetime: >10 years at operating conditions

#### 4.1.3 C4 Bumps (Interposer to Substrate)

**Bump Specifications**:
- Pitch: 130-180 μm
- Bump height: 50-80 μm after reflow
- Bump diameter: 80-120 μm
- Coplanarity: ±25 μm across interposer

**Materials**:
- Lead-free solder: SAC305, SAC405
- UBM: Cu/Ni(V)/Au or similar

#### 4.1.4 Package Substrate

**Specifications**:
- Layers: 8-20 metal layers
- Core thickness: 200-800 μm
- Build-up layers: 2-10 per side
- Line/space in build-up: 15/15 μm minimum
- Via capture pad: ≥50 μm diameter

**Materials**:
- Core: BT resin, ABF, or similar
- Copper thickness: 12-35 μm
- Surface finish: ENIG, ENEPIG, or OSP

### 4.2 3D Packaging Physical Specifications

#### 4.2.1 Die Stacking

**Vertical Alignment**:
- Die-to-die alignment accuracy: ±2 μm (face-to-face bonding)
- Die-to-die alignment accuracy: ±5 μm (face-to-back bonding)

**Die Thinning**:
- Thinned die thickness: 10-100 μm
- Thickness variation: <±2 μm across die
- Total thickness variation: <±3 μm across wafer
- Surface roughness: <0.5 nm Ra (for hybrid bonding)

#### 4.2.2 Hybrid Bonding

**Bond Interface**:
- Copper pad pitch: 1-10 μm
- Copper recess: 0-50 nm relative to dielectric
- Surface roughness: <0.5 nm Ra
- Particle requirements: Zero particles >50 nm
- Overlay accuracy: <200 nm

**Bond Quality**:
- Void area: <1% of total bond area
- Bond strength: ≥1 MPa (dielectric), ≥50 MPa (copper)
- Electrical resistance: <1 mΩ·μm² (specific contact resistance)

#### 4.2.3 HBM Stack Specifications

**Stack Configuration**:
- DRAM dies per stack: 4-12 layers
- Die thickness: 25-50 μm
- Total stack height: 200-720 μm
- Logic base die thickness: 200-300 μm

**TSV Specifications (HBM)**:
- Diameter: 5-10 μm
- Pitch: 40-55 μm
- TSV count per die: 5,000-10,000
- Resistance: <100 mΩ per TSV

**Thermal Properties**:
- Thermal resistance (junction to base die): <0.05 °C/W per layer
- Maximum temperature gradient: <5 °C/mm

### 4.3 Fan-Out Packaging Physical Specifications

#### 4.3.1 Package Dimensions

**Overall Package**:
- Thickness: 0.3-1.0 mm
- Warpage: <200 μm at room temperature, <400 μm at reflow temperature
- Die placement accuracy: ±10 μm
- Reconstituted wafer diameter: 300 mm standard

**Molding Compound Coverage**:
- Minimum thickness above die: 50-150 μm
- Minimum thickness below die: 50-200 μm
- Mold flash: <10 μm

#### 4.3.2 RDL Specifications (Fan-Out)

**Metal Layers**:
- Number of layers: 1-6
- Line width/spacing: 2/2 μm minimum
- Metal thickness: 3-8 μm per layer
- Via diameter: 10-30 μm

**Dielectric**:
- Material: Polyimide, PBO, or photosensitive polymer
- Thickness: 5-15 μm per layer
- CTE: 15-40 ppm/°C

**Passivation Layer**:
- Thickness: 5-15 μm
- Opening accuracy: ±10 μm

#### 4.3.3 Under-Bump Metallurgy (Fan-Out)

**UBM Stack**:
- Layers: Ti/Cu or Ti/Ni(V)/Cu
- Total thickness: 0.5-3 μm
- Pad size: 150-400 μm diameter

**Solder Ball Specifications**:
- Ball diameter: 300-600 μm
- Pitch: 0.4-0.8 mm
- Solder alloy: SAC305 or similar lead-free
- Coplanarity: ±50 μm

### 4.4 Chiplet Interconnect Physical Specifications

#### 4.4.1 UCIe Physical Layer

**Standard Package (UCIe-S)**:
- Bump pitch: 25 μm or 55 μm
- Signaling: Single-ended or differential
- Die-to-die spacing: 0.5-10 mm
- Trace impedance: 45-55 Ω (single-ended), 90-110 Ω (differential)

**Advanced Package (UCIe-A)**:
- Bump pitch: 9 μm minimum (roadmap to finer)
- Die-to-die spacing: <1 mm
- Optimized for ultra-short reach

**Electrical Properties**:
- Insertion loss: <3 dB at Nyquist frequency
- Return loss: >10 dB at Nyquist frequency
- Crosstalk: <-30 dB
- ISI: <0.25 UI

## 5. Material Specifications

### 5.1 Solder Materials

**SAC305 (Sn-3.0Ag-0.5Cu)**:
- Melting point: 217-220°C
- Yield strength: 35-40 MPa
- Ultimate tensile strength: 45-50 MPa
- Thermal conductivity: 60 W/m·K

**SAC405 (Sn-4.0Ag-0.5Cu)**:
- Melting point: 217-219°C
- Yield strength: 40-45 MPa (higher than SAC305)

### 5.2 Molding Compound

**Properties**:
- CTE: 10-15 ppm/°C (below Tg), 25-35 ppm/°C (above Tg)
- Tg (Glass transition temperature): >150°C
- Thermal conductivity: 0.5-5 W/m·K (depending on filler)
- Flexural modulus: 15-25 GPa
- Moisture absorption: <0.3% (24 hours at 85°C/85% RH)
- Filler content: 70-90% (typically silica)

### 5.3 Underfill Materials

**Properties**:
- CTE: 20-40 ppm/°C
- Tg: >120°C
- Thermal conductivity: 0.3-1.0 W/m·K
- Viscosity: 1,000-50,000 cP at dispensing temperature
- Cure temperature: 130-180°C
- Cure time: 15-60 minutes

### 5.4 Thermal Interface Materials

**Requirements**:
- Thermal conductivity: 1-80 W/m·K (depending on TIM type)
- Bond line thickness: 25-500 μm
- Thermal resistance: <0.05 °C·cm²/W
- Operating temperature: -40 to 150°C

**Types**:
- Thermal greases: 1-5 W/m·K
- Phase change materials: 2-8 W/m·K
- Thermal pads: 1-5 W/m·K
- Solder TIMs: 20-80 W/m·K
- Liquid metal: 20-70 W/m·K

## 6. Mechanical Requirements

### 6.1 Warpage Limits

**At Room Temperature (25°C)**:
- 2.5D packages: <200 μm
- Fan-out packages: <200 μm
- Individual dies before assembly: <50 μm

**At Reflow Temperature (260°C)**:
- Maximum warpage: <500 μm
- Die shift during reflow: <10 μm

### 6.2 Adhesion Requirements

**Die Adhesion**:
- Shear strength: ≥5 MPa
- No delamination after reliability testing

**RDL Adhesion**:
- Cross-hatch adhesion: 5B rating (ASTM D3359)
- Peel strength: ≥0.7 N/mm

### 6.3 Stress Limits

**Die Stress**:
- Maximum von Mises stress: <300 MPa at room temperature
- Keep-out zones around TSVs: 5-20 μm depending on TSV diameter

**Package Stress**:
- No visible cracks or delamination after thermal cycling
- Acceptable stress concentration factors depend on design

## 7. Thermal Requirements

### 7.1 Thermal Resistance

**Package-Level**:
- θJC (Junction to Case): 0.1-1.0 °C/W depending on package type and power
- θJB (Junction to Board): 5-20 °C/W depending on board design
- θJA (Junction to Ambient): System-dependent

**Material Thermal Conductivity**:
- Silicon: ≥130 W/m·K
- Copper: ≥390 W/m·K
- Molding compound: 0.5-5 W/m·K
- Underfill: 0.3-1.0 W/m·K

### 7.2 Operating Temperature

**Junction Temperature**:
- Maximum: 125°C for commercial, 150°C for industrial
- Continuous operating: 105°C typical

**Package Operating Range**:
- Commercial: 0 to 85°C
- Industrial: -40 to 105°C
- Automotive: -40 to 125°C (AEC-Q100 Grade 2)
- Military: -55 to 125°C

## 8. Electrical Requirements

### 8.1 Resistance

**Interconnect Resistance**:
- TSV: <50 mΩ per via
- Microbump: <10 mΩ per bump
- C4 bump: <5 mΩ per bump
- RDL trace: <0.1 Ω/mm for power, <1 Ω/mm for signal

### 8.2 Capacitance and Inductance

**Parasitic Limits**:
- TSV capacitance: <10 fF
- Bump inductance: <10 pH per bump
- Package trace capacitance: Design-dependent

### 8.3 Isolation

**Electrical Isolation**:
- TSV to substrate: >100 GΩ at 125°C
- Die-to-die isolation (3D): >10 GΩ
- Breakdown voltage: >100 V for standard applications

## 9. Reliability Requirements

### 9.1 Qualification Testing

All packages must pass the following tests per JEDEC standards:
- Temperature Cycling: -40 to 125°C, 1000 cycles minimum
- High Temperature Storage Life: 150°C, 1000 hours
- Temperature Humidity Bias: 85°C/85% RH, 1000 hours
- Highly Accelerated Stress Test: 130°C/85% RH, 96 hours
- Preconditioning per MSL rating
- Board-level reliability: 0 to 100°C, 1500 cycles

### 9.2 Mean Time To Failure (MTTF)

**Minimum Requirements**:
- Consumer: ≥5 years at use conditions
- Industrial: ≥10 years
- Automotive: ≥15 years
- Based on acceleration factors and qualification test results

### 9.3 Electrostatic Discharge (ESD)

**ESD Protection Levels**:
- Human Body Model (HBM): ≥2 kV
- Charged Device Model (CDM): ≥500 V
- Machine Model (MM): ≥200 V (if applicable)

## 10. Inspection and Testing

### 10.1 Visual Inspection

**Defect Criteria**:
- No cracks, chips, or delamination visible
- No exposed die or bond wires
- No contamination on bonding surfaces

### 10.2 X-Ray Inspection

**Acceptance Criteria**:
- Solder voids: <30% of bump area
- No missing bumps
- No bridging between adjacent bumps
- Proper die and component placement

### 10.3 Acoustic Microscopy

**Delamination Limits**:
- No delamination >25% of interface area
- No cracks extending >500 μm

### 10.4 Electrical Testing

**Test Requirements**:
- Continuity testing of all connections
- Shorts testing between adjacent nets
- Parametric testing of critical specifications
- Functional testing at speed

## 11. Documentation Requirements

### 11.1 Design Documentation

Required documentation includes:
- Package drawing with dimensions and tolerances
- Materials list with specifications
- Cross-section showing all layers and materials
- Thermal simulation results
- Electrical simulation results (S-parameters, power integrity)

### 11.2 Manufacturing Documentation

Required documentation includes:
- Process flow diagram
- Critical process parameters and control limits
- Inspection and test procedures
- Qualification test reports

## 12. Compliance

Conformance to this specification shall be demonstrated through:
- Design review verifying compliance with dimensional and material requirements
- Qualification testing per Section 9
- Manufacturing process capability studies
- Ongoing production monitoring

---

**© 2025 SmileStory Inc. / WIA**
弘益人間 (홍익인간) · Benefit All Humanity
