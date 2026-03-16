# WIA-SPACE-016: Aircraft Component Standard v1.0

**Standard ID:** WIA-SPACE-016
**Version:** 1.0.0
**Status:** Published
**Published:** 2025-12-26
**Category:** Space & Aviation
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Abstract

The WIA-SPACE-016 Aircraft Component Standard provides comprehensive specifications for the design, manufacturing, testing, certification, and maintenance of aircraft components. This standard ensures safety, reliability, and interoperability across the global aviation industry by establishing unified requirements for structural components, materials, systems, and quality management.

### 1.1 Scope

This standard covers:
- **Structural Components:** Fuselage, wings, tail assemblies, and primary structures
- **Materials:** Aluminum alloys, composite materials, titanium, and advanced materials
- **Landing Gear:** Design, shock absorption, braking, and retraction systems
- **Flight Controls:** Primary and secondary control surfaces, fly-by-wire systems
- **Hydraulic & Fuel Systems:** Hydraulic circuits, fuel storage, distribution, management
- **Avionics:** Navigation, communication, surveillance, flight management
- **Certification:** AS9100, NADCAP, FAA/EASA compliance
- **Lifecycle Management:** Maintenance, inspection, fatigue life, corrosion prevention

### 1.2 Target Audience

- Aircraft manufacturers and OEMs
- Component suppliers and subcontractors
- Certification authorities (FAA, EASA, etc.)
- Maintenance, repair, and overhaul (MRO) organizations
- Engineering teams and quality assurance personnel
- Aviation safety regulators

---

## 2. Aircraft Structure Standards

### 2.1 Fuselage Structure

#### 2.1.1 Semi-Monocoque Design
- **Frames:** Circular or semi-circular structures spaced 15-20 inches apart
- **Stringers:** Longitudinal stiffeners preventing skin buckling
- **Skin:** Aluminum or composite panels, thickness 0.040-0.125 inches
- **Bulkheads:** Pressure-resistant partitions at critical stations
- **Floor Beams:** Support cabin floor and distribute loads

#### 2.1.2 Pressurization Requirements
- **Cabin Pressure Differential:** 8.0 psi (0.55 bar) maximum
- **Safety Factor:** 2.0 for pressure loads
- **Proof Pressure:** 1.33 × maximum operating differential
- **Ultimate Pressure:** 2.0 × maximum operating differential
- **Fatigue Life:** 60,000 pressurization cycles minimum

### 2.2 Wing Structure

#### 2.2.1 Wing Box Construction
- **Spars:** Front and rear spars carrying bending moments
- **Ribs:** Maintain airfoil shape, spaced 12-24 inches
- **Skin:** Upper and lower wing skins resisting shear and torsion
- **Stringers:** Longitudinal stiffeners on wing skins
- **Wing Box:** Closed structure formed by spars and skins

#### 2.2.2 Load Factors (FAR Part 25)
- **Positive Limit Load:** +2.5g at maximum landing weight
- **Positive Ultimate Load:** +3.75g (1.5 × limit load)
- **Negative Limit Load:** -1.0g
- **Gust Loads:** 50 fps vertical gust at cruise speed

### 2.3 Tail Assembly

#### 2.3.1 Horizontal Stabilizer
- **Configuration:** Fixed stabilizer with movable elevator OR all-moving stabilizer
- **Span:** Typically 25-30% of wing span
- **Area:** 15-20% of wing area
- **Load Factors:** ±3.75g ultimate load

#### 2.3.2 Vertical Stabilizer
- **Area:** 10-15% of wing area
- **Rudder:** 25-30% of vertical stabilizer area
- **Side Load:** Withstand crosswind landing and asymmetric thrust

---

## 3. Material Standards

### 3.1 Aluminum Alloys

#### 3.1.1 2024-T3 (Fuselage Skin)
```
Chemical Composition:
- Copper (Cu): 3.8-4.9%
- Magnesium (Mg): 1.2-1.8%
- Manganese (Mn): 0.3-0.9%
- Remainder: Aluminum

Mechanical Properties:
- Ultimate Tensile Strength: 470 MPa (68 ksi)
- Yield Strength: 325 MPa (47 ksi)
- Elongation: 18%
- Elastic Modulus: 73 GPa
- Density: 2.78 g/cm³

Fatigue Properties:
- Fatigue Strength (10⁷ cycles): 140 MPa
- Fracture Toughness (KIc): 26 MPa√m
```

#### 3.1.2 7075-T6 (High-Strength Structure)
```
Chemical Composition:
- Zinc (Zn): 5.1-6.1%
- Magnesium (Mg): 2.1-2.9%
- Copper (Cu): 1.2-2.0%
- Remainder: Aluminum

Mechanical Properties:
- Ultimate Tensile Strength: 570 MPa (83 ksi)
- Yield Strength: 505 MPa (73 ksi)
- Elongation: 11%
- Elastic Modulus: 72 GPa
- Density: 2.81 g/cm³
```

### 3.2 Composite Materials

#### 3.2.1 Carbon Fiber/Epoxy (Primary Structure)
```
Fiber:
- Type: T700 or IM7 carbon fiber
- Tensile Strength: 4,900 MPa
- Elastic Modulus: 230 GPa
- Density: 1.80 g/cm³

Matrix:
- Type: Epoxy resin (e.g., 3501-6, 977-3)
- Cure Temperature: 120-180°C
- Glass Transition Temp: 170-220°C

Laminate Properties (Quasi-Isotropic):
- Tensile Strength: 600-800 MPa
- Compressive Strength: 500-600 MPa
- Shear Strength: 100-120 MPa
- Elastic Modulus: 60-70 GPa
- Density: 1.55-1.60 g/cm³
```

#### 3.2.2 Laminate Design
- **0° Plies:** 25% (longitudinal strength)
- **±45° Plies:** 50% (shear strength)
- **90° Plies:** 25% (transverse strength)
- **Stacking Sequence:** [45/0/-45/90]s (symmetric and balanced)

### 3.3 Titanium Alloys

#### 3.3.1 Ti-6Al-4V (Landing Gear, Engine Mounts)
```
Chemical Composition:
- Aluminum (Al): 5.5-6.75%
- Vanadium (V): 3.5-4.5%
- Remainder: Titanium

Mechanical Properties:
- Ultimate Tensile Strength: 950 MPa (138 ksi)
- Yield Strength: 880 MPa (128 ksi)
- Elongation: 14%
- Elastic Modulus: 114 GPa
- Density: 4.43 g/cm³
- Maximum Service Temp: 400°C
```

---

## 4. Landing Gear Standards

### 4.1 Design Requirements

#### 4.1.1 Load Conditions
- **Static Load:** Maximum taxi weight
- **Dynamic Load:** 2.5-3.0 × static load (landing impact)
- **Braking Load:** 1.0g deceleration
- **Side Load:** 0.3g lateral (crosswind)
- **Spin-up Load:** Tire rotation acceleration

#### 4.1.2 Shock Absorber
```
Oleo-Pneumatic Strut:
- Type: Gas-oil shock absorber
- Gas: Nitrogen (N₂)
- Hydraulic Fluid: MIL-PRF-5606
- Static Pressure: 2,500 psi (172 bar)
- Maximum Pressure: 4,500 psi (310 bar)
- Stroke: 300-500 mm
- Energy Absorption: 80-90%
```

### 4.2 Tire Requirements

```
Tire Specifications (Example: B737):
- Size: 40×14
- Ply Rating: 20-26 plies
- Inflation Pressure: 200 psi (13.8 bar)
- Speed Rating: 160 knots (296 km/h)
- Load Rating: 25,000 lbs (11,340 kg) per tire
- Tread Depth (New): 16-18 mm
- Minimum Tread Depth: 1.6 mm
- Service Life: 200-300 landings
```

### 4.3 Braking System

#### 4.3.1 Carbon Brake Performance
```
Material: Carbon-Carbon Composite
- Working Temperature: 1,000-1,500°C
- Friction Coefficient: 0.3-0.5 (temperature dependent)
- Mass: 40% lighter than steel brakes
- Service Life: 2,500-3,000 landings
- Kinetic Energy Capacity: 100-150 MJ per brake

Anti-Skid System:
- Response Time: < 50 ms
- Slip Ratio Control: 10-20% optimal slip
- Wheel Speed Sampling: 100 Hz
- Independent Control: Each wheel individually controlled
```

---

## 5. Flight Control Standards

### 5.1 Primary Flight Controls

#### 5.1.1 Control Surface Sizing
```
Ailerons:
- Span: 15-25% of wing semi-span
- Chord: 20-30% of wing chord
- Deflection: ±25° typical
- Hinge Moment: 5,000-15,000 N⋅m

Elevators:
- Area: 15-20% of horizontal stabilizer area
- Deflection: +25°/-15° typical
- Hinge Moment: 10,000-30,000 N⋅m

Rudder:
- Area: 25-30% of vertical stabilizer area
- Deflection: ±25° typical
- Hinge Moment: 8,000-25,000 N⋅m
```

### 5.2 Fly-by-Wire System

#### 5.2.1 Architecture
- **Flight Control Computers:** Triple or quadruple redundancy
- **Data Bus:** ARINC 429 or ARINC 664 (AFDX)
- **Actuators:** Electro-hydraulic or electro-mechanical
- **Sensors:** Position, rate, acceleration (triple redundant)
- **Power:** Independent power sources for each channel

#### 5.2.2 Performance Requirements
```
System Response:
- Command to Surface Movement: < 100 ms
- Position Accuracy: ±0.5°
- Rate Capability: 50-100°/s
- Failure Detection: < 20 ms
- Fault Recovery: < 200 ms

Control Laws:
- Normal Law: Full envelope protection
- Alternate Law: Reduced protections
- Direct Law: Mechanical backup feel
```

---

## 6. Hydraulic and Fuel System Standards

### 6.1 Hydraulic System

#### 6.1.1 System Specifications
```
Operating Pressure: 3,000 psi (207 bar)
- Proof Pressure: 4,500 psi (310 bar)
- Burst Pressure: 9,000 psi (620 bar)

Hydraulic Fluid:
- Type: MIL-PRF-5606 or MIL-PRF-83282
- Operating Temp: -54°C to +135°C
- Viscosity: 13.5-15.5 cSt @ 40°C

System Redundancy:
- Primary: System A + System B
- Backup: System C (electric pump)
- Emergency: Ram Air Turbine (RAT)
```

#### 6.1.2 Pump Requirements
```
Engine-Driven Pump (EDP):
- Type: Variable displacement piston pump
- Flow Rate: 20-40 gpm (75-150 L/min)
- Efficiency: > 85%
- MTBF: > 10,000 flight hours

Electric Motor Pump (EMP):
- Type: Fixed displacement or variable
- Power: 7.5-15 kW
- Flow Rate: 10-20 gpm (38-75 L/min)
- Operating Time: Continuous or intermittent
```

### 6.2 Fuel System

#### 6.2.1 Fuel Tank Design
```
Tank Types:
- Integral Tanks: Sealed wing structure
- Bladder Tanks: Flexible fuel cells
- Rigid Tanks: Aluminum alloy containers

Fuel Capacity (Example: B737):
- Wing Tanks: 46,534 lbs (21,100 kg)
- Center Tank: 19,160 lbs (8,690 kg)
- Total Usable Fuel: 65,694 lbs (29,790 kg)

Ullage: 2-3% of tank volume (fuel expansion)
Vent System: Prevent overpressure and vacuum
```

#### 6.2.2 Fuel Management
```
Fuel Feed System:
- Boost Pumps: 2 per tank (redundancy)
- Suction Feed: Backup (gravity/suction)
- Flow Rate: 5,000-10,000 lbs/hr per engine

Center of Gravity Control:
- Forward CG Limit: 15% MAC
- Aft CG Limit: 30% MAC
- Fuel Sequencing: Maintain CG within limits

Fuel Jettison (if equipped):
- Jettison Rate: 2,000-4,000 lbs/min
- Jettison to: Maximum landing weight
- Safety Height: > 5,000 ft AGL
```

---

## 7. Avionics Standards

### 7.1 Navigation Systems

#### 7.1.1 Integrated Navigation
```
GPS (Global Positioning System):
- Accuracy: < 10 m horizontal (GPS alone)
- Accuracy: < 1 m (WAAS/SBAS augmented)
- Update Rate: 1-10 Hz
- Constellation: GPS + GLONASS + Galileo

INS (Inertial Navigation System):
- Drift Rate: < 1 nm/hr (laser gyro)
- Drift Rate: < 0.01 nm/hr (ring laser gyro)
- Alignment Time: 5-10 minutes

FMS (Flight Management System):
- Route Waypoints: 500+ waypoints
- Performance Database: Aircraft-specific
- Vertical Navigation: VNAV guidance
- Lateral Navigation: LNAV guidance
```

### 7.2 Communication Systems

#### 7.2.1 Radio Systems
```
VHF Communication:
- Frequency Range: 118-137 MHz
- Channel Spacing: 8.33 kHz (25 kHz legacy)
- Number of Radios: 2-3 (redundancy)
- Range: Line of sight (200-300 nm at cruise)

HF Communication:
- Frequency Range: 2-30 MHz
- Range: Worldwide (long distance)
- Data Rate: Low (voice and ACARS)

SATCOM (Satellite Communication):
- System: Inmarsat or Iridium
- Data Rate: 10-100 kbps
- Coverage: Global (including polar)
- Services: Voice, data, internet
```

### 7.3 Surveillance Systems

#### 7.3.1 Collision Avoidance
```
TCAS (Traffic Collision Avoidance System):
- Version: TCAS II (current standard)
- Detection Range: 40 nm
- Altitude Resolution: ±100 ft
- Traffic Advisories (TA): 40 seconds warning
- Resolution Advisories (RA): 25 seconds warning

ADS-B (Automatic Dependent Surveillance-Broadcast):
- Frequency: 1090 MHz (Mode S Extended Squitter)
- Update Rate: 1 Hz (position)
- Accuracy: GPS-based (< 10 m)
- Mandate: Required in controlled airspace (2020+)
```

---

## 8. Certification Standards

### 8.1 AS9100 Quality Management

#### 8.1.1 Requirements
- **Quality Management System:** ISO 9001 + aerospace-specific requirements
- **Configuration Management:** Control of design changes and versions
- **Risk Management:** Identification and mitigation of risks
- **Supplier Management:** Approved supplier list and audits
- **Continuous Improvement:** Corrective and preventive actions (CAPA)

#### 8.1.2 Documentation
- **Quality Manual:** Top-level quality policy and objectives
- **Procedures:** Work instructions and process documentation
- **Records:** Traceability of materials, processes, inspections
- **Audits:** Internal audits (annually), external audits (certification body)

### 8.2 NADCAP Certification

#### 8.2.1 Special Processes
```
Covered Processes:
- Welding: Resistance, TIG, MIG, electron beam, laser
- Heat Treatment: Annealing, hardening, tempering, stress relief
- Non-Destructive Testing: X-ray, ultrasonic, eddy current, penetrant
- Chemical Processing: Anodizing, plating, chemical milling
- Composite Materials: Lay-up, curing, bonding, repair

Audit Frequency:
- Initial Audit: Before certification
- Surveillance Audit: Every 6-12 months (process dependent)
- Re-certification: Every 2-3 years
```

### 8.3 FAA/EASA Type Certification

#### 8.3.1 Certification Basis
- **FAR Part 25:** Transport Category Airplanes
- **CS-25:** EASA Certification Specifications (equivalent to FAR 25)
- **Advisory Circulars:** Acceptable means of compliance
- **Special Conditions:** For novel or unusual design features

#### 8.3.2 Compliance Methods
- **Analysis:** Engineering calculations and simulations
- **Testing:** Ground tests, flight tests, certification tests
- **Similarity:** Reference to previously approved designs
- **Service Experience:** Operational data from similar aircraft

---

## 9. Maintenance and Lifecycle Standards

### 9.1 Maintenance Program

#### 9.1.1 Check Intervals
```
A Check:
- Interval: 500-800 flight hours or 200-300 flights
- Duration: 10-20 hours
- Location: Line maintenance (airport)
- Scope: Visual inspections, lubrication, minor repairs

C Check:
- Interval: 18-24 months or 3,000-4,500 flight hours
- Duration: 1-2 weeks
- Location: Hangar maintenance
- Scope: Detailed inspections, NDT, system tests, component replacement

D Check:
- Interval: 6-10 years or 30,000-60,000 flight hours
- Duration: 1-2 months
- Location: Major maintenance facility
- Scope: Complete structural inspection, overhaul, modifications
```

### 9.2 Non-Destructive Inspection (NDI)

#### 9.2.1 Inspection Methods
```
Eddy Current Inspection:
- Application: Surface cracks in aluminum, titanium
- Depth: 0-5 mm below surface
- Crack Detection: > 0.5 mm length
- Frequency: 10 kHz - 10 MHz

Ultrasonic Inspection:
- Application: Internal defects, corrosion, thickness
- Depth: 0-500 mm (depends on material)
- Resolution: 0.5 mm
- Method: Pulse-echo, through-transmission

Radiographic Inspection:
- Application: Internal voids, inclusions, cracks
- Source: X-ray (portable) or gamma-ray (Co-60, Ir-192)
- Sensitivity: 1-2% of material thickness
- Recording: Film or digital detector
```

### 9.3 Fatigue Life Management

#### 9.3.1 Damage Tolerance
```
Inspection Program:
- Initial Inspection Threshold: 50% of calculated life
- Repeat Interval: Every 10,000 flights or as calculated
- Crack Growth Monitoring: Track crack size over time
- Retirement Life: Before crack reaches critical size

Structural Monitoring:
- Load Monitoring: Record flight loads (FOQA data)
- Usage Monitoring: Cycles, hours, exceedances
- Structural Health Monitoring: Sensors on critical structure
- Fleet Leader: Most severely used aircraft inspected first
```

---

## 10. Conformity and Compliance

### 10.1 Design Verification

All aircraft components shall be verified through:
1. **Analysis:** Finite element analysis (FEA), computational fluid dynamics (CFD)
2. **Testing:** Static tests, fatigue tests, environmental tests
3. **Inspection:** Dimensional checks, material verification, NDI
4. **Demonstration:** Functional tests, flight tests

### 10.2 Traceability

- **Material Traceability:** Certificate of conformance for all materials
- **Process Traceability:** Records of manufacturing processes and parameters
- **Inspection Traceability:** Inspection reports and test results
- **Serial Number Tracking:** Unique identification for each component

---

## 11. References and Standards

### 11.1 Regulatory Documents
- **FAR Part 21:** Certification Procedures for Products and Articles
- **FAR Part 25:** Airworthiness Standards: Transport Category Airplanes
- **EASA CS-25:** Certification Specifications for Large Aeroplanes
- **AC 20-107B:** Composite Aircraft Structure
- **AC 25.571-1D:** Damage Tolerance and Fatigue Evaluation

### 11.2 Industry Standards
- **AS9100:** Quality Management Systems - Aerospace
- **AS9102:** Aerospace First Article Inspection Requirement
- **NADCAP:** National Aerospace and Defense Contractors Accreditation Program
- **ASTM F3301:** Additive Manufacturing for Aerospace Applications
- **RTCA DO-160:** Environmental Conditions and Test Procedures for Airborne Equipment

### 11.3 Material Specifications
- **AMS 4037:** Aluminum Alloy 2024-T3
- **AMS 4045:** Aluminum Alloy 7075-T6
- **AMS 4911:** Titanium Alloy Ti-6Al-4V
- **ASTM D3039:** Tensile Properties of Polymer Matrix Composites
- **MIL-HDBK-5J:** Metallic Materials and Elements for Aerospace Vehicle Structures

---

## 12. Revision History

| Version | Date       | Changes                          | Author      |
|---------|------------|----------------------------------|-------------|
| 1.0.0   | 2025-12-26 | Initial release                  | WIA Standards Committee |

---

**Published by:**
World Certification Industry Association (WIA)
弘益人間 · Benefit All Humanity

**Contact:**
Website: https://wiastandards.com
Email: standards@wiastandards.com
GitHub: https://github.com/WIA-Official/wia-standards

**License:**
MIT License
© 2025 WIA Standards. All rights reserved.
