# WIA-SEMI-020: Thermal Management Specification

## 1. Scope
This specification defines thermal management requirements, test methods, and design guidelines for advanced semiconductor packaging.

## 2. Thermal Resistance Specifications

### 2.1 Package Thermal Resistance
- θJC: 0.1-1.0 °C/W for high-performance packages
- θJB: 5-20 °C/W depending on PCB design
- θJA: System and cooling dependent

### 2.2 Measurement Methods
Thermal resistance measured per JEDEC JESD51 series standards using thermal test dies with integrated heaters and temperature sensors.

## 3. Junction Temperature Limits

### 3.1 Maximum Ratings
- Consumer/Commercial: 125°C maximum
- Industrial: 150°C maximum
- Automotive (AEC-Q100): 
  - Grade 0: 150-175°C
  - Grade 1: 125-150°C
  - Grade 2: 105-125°C

### 3.2 Continuous Operating Limits
Typical continuous junction temperature: 105°C to ensure reliability over product lifetime.

## 4. Thermal Interface Materials (TIM) Requirements

### 4.1 TIM Properties
- Thermal conductivity: 1-80 W/m·K depending on application
- Bond line thickness: 25-500 μm
- Thermal resistance: <0.05 °C·cm²/W
- Operating temperature range: -40 to 150°C
- Pump-out resistance: <10% thickness change after 1000 thermal cycles

### 4.2 TIM Types and Applications
**Thermal Greases/Pastes**: 1-5 W/m·K, easy application, moderate performance
**Phase Change Materials**: 2-8 W/m·K, solid at room temp, flows during operation
**Thermal Pads**: 1-5 W/m·K, pre-formed, consistent thickness
**Solder TIM**: 20-80 W/m·K, excellent performance, difficult rework
**Liquid Metal TIM**: 20-70 W/m·K, highest performance, requires containment

## 5. Heat Sink Requirements

### 5.1 Heat Sink Performance
Heat sink thermal resistance: 0.1-5 °C/W depending on size, airflow, and cooling method.

### 5.2 Heat Sink Attachment
- Mounting pressure: 30-150 psi
- Attachment method: Screw-down, clip, or adhesive
- Coplanarity: ±50 μm across contact area
- Surface finish: <1.6 μm Ra

### 5.3 Heat Sink Materials
- Aluminum: Low cost, thermal conductivity ~200 W/m·K
- Copper: Higher performance, thermal conductivity ~400 W/m·K
- Vapor chamber/heat pipe: Very high effective thermal conductivity (>1000 W/m·K effective)

## 6. Package Thermal Design

### 6.1 2.5D Thermal Design
**Heat Paths**: Through silicon interposer to substrate, and top surface to heat sink/spreader.
**Thermal Vias**: Use thermal vias in substrate (diameter ≥250 μm, spacing ≤500 μm) to improve heat extraction.
**Die Placement**: Optimize die placement to avoid thermal hotspots, maintain >2mm spacing between high-power dies when possible.

### 6.2 3D Thermal Design
**Vertical Heat Flow**: Each stacked die adds thermal resistance; limit stack height based on power budget.
**TSV Thermal Enhancement**: Use dedicated thermal TSVs (>10% of total TSV area recommended).
**Die Thinning**: Thinner dies (30-50 μm) reduce thermal resistance compared to thicker dies (100 μm+).
**Thermal Management**: Consider active cooling (fans, liquid cooling) for high-power 3D stacks (>100W total).

### 6.3 Fan-Out Thermal Design
**Direct Heat Path**: Die backside exposed or with thin molding provides direct thermal path to heat sink.
**Molding Compound**: Use high thermal conductivity molding compounds (2-5 W/m·K) for improved performance.
**RDL Thermal Design**: Maximize copper density in RDL layers for heat spreading; use thermal vias where appropriate.

## 7. Thermal Simulation Requirements

### 7.1 Simulation Tools
Use finite element analysis (FEA) tools capable of multi-physics thermal simulation: ANSYS Mechanical, Siemens FloTHERM, Cadence Celsius, or equivalent.

### 7.2 Model Requirements
**Geometry**: Accurate 3D geometry including all package layers, dies, substrate, TIM, heat sink.
**Materials**: Temperature-dependent material properties where significant.
**Power Maps**: Realistic die power maps from electrical simulation or measurement.
**Boundary Conditions**: Appropriate ambient temperature, convective heat transfer coefficients, airflow.

### 7.3 Validation
Thermal simulations must be validated against physical measurements using thermal test dies or IR imaging within ±10% agreement.

## 8. Thermal Testing and Characterization

### 8.1 Thermal Test Die
Thermal test dies shall include:
- Resistive heaters covering >50% of die area
- Temperature sensors (diodes, resistors) at multiple locations
- Power and temperature sense connections

### 8.2 Junction-to-Case Thermal Resistance (θJC)
Measured per JEDEC JESD51-2:
- Mount package on cold plate at controlled temperature
- Apply known power to test die
- Measure junction temperature rise
- θJC = ΔT / P

### 8.3 Junction-to-Ambient Thermal Resistance (θJA)
Measured per JEDEC JESD51-2A in still air:
- Mount package on test board in still air chamber
- Apply known power
- Measure junction and ambient temperatures
- θJA = (TJ - TA) / P

### 8.4 Thermal Transient Testing
Measure thermal time constants and package structure using thermal transient testing per JEDEC JESD51-14.

### 8.5 Infrared Thermal Imaging
Use IR camera to measure surface temperature distributions:
- Calibrate for emissivity variations
- Compare to simulation results
- Identify hotspots and thermal gradients

## 9. Thermal Reliability

### 9.1 Temperature Cycling Effects
Thermal cycling induces stress due to CTE mismatch:
- Silicon: 2.6 ppm/°C
- Molding compound: 10-30 ppm/°C
- Substrate: 15-18 ppm/°C
- Solder: 20-25 ppm/°C

### 9.2 Electromigration
Electromigration lifetime follows Black's equation; temperature critically affects lifetime (doubles approximately every 10°C reduction).

### 9.3 Time-Dependent Dielectric Breakdown (TDDB)
TDDB follows Arrhenius temperature dependence with activation energy typically 0.3-1.5 eV.

## 10. Cooling Solutions

### 10.1 Air Cooling
**Natural Convection**: h = 5-25 W/m²·K, suitable for low power (<10W packages).
**Forced Air**: h = 25-250 W/m²·K with fan; performance depends on airflow rate and fin design.

### 10.2 Liquid Cooling
**Cold Plates**: Heat exchanger attached to package with liquid flowing through internal channels; h = 500-10,000 W/m²·K.
**Immersion Cooling**: Package submerged in dielectric fluid; excellent thermal performance for high-density systems.
**Microfluidic Cooling**: Microchannels integrated in package or die; can achieve h > 10,000 W/m²·K; enables >1000 W/cm² heat flux.

### 10.3 Advanced Cooling Technologies
**Vapor Chambers**: Spreads heat efficiently; effective thermal conductivity >1000 W/m·K
**Heat Pipes**: Transports heat over distance with minimal temperature drop
**Thermoelectric Coolers**: Peltier effect cooling; can achieve sub-ambient temperatures but power-hungry (COP typically <1)
**Phase Change Materials**: Provides thermal buffering during transient loads

## 11. Design Guidelines

### 11.1 Thermal-Aware Floorplanning
- Distribute high-power blocks across die area
- Avoid concentration of power dissipation
- Place highest power blocks nearest to best thermal path
- Maintain thermal guard bands around temperature-sensitive circuits

### 11.2 Power Management
- Implement dynamic voltage and frequency scaling (DVFS)
- Use power gating for unused blocks
- Thermal throttling as last-resort protection
- Monitor junction temperature with on-die sensors

### 11.3 Package Selection
Select package type based on thermal requirements:
- Low power (<5W): Standard packages with natural convection adequate
- Medium power (5-50W): Enhanced packages with heat spreaders or small heat sinks
- High power (50-200W): 2.5D or advanced packages with substantial heat sinks and forced air
- Very high power (>200W): Specialized cooling (liquid cooling, high-performance heat sinks)

## 12. Thermal Specifications Summary Table

| Package Type | θJC (°C/W) | Max Power (W) | Cooling Method | TIM Required |
|-------------|-----------|--------------|----------------|--------------|
| Standard BGA | 2-5 | 5-20 | Natural/Forced air | Optional |
| Enhanced BGA | 0.5-2 | 20-80 | Forced air + heatsink | Yes |
| 2.5D with HBM | 0.2-0.8 | 100-300 | Large heatsink + fan | Yes |
| Fan-out WLP | 1-3 | 10-40 | Heat spreader | Yes |
| 3D Stack | 0.5-2 | 30-150 | Advanced cooling | Yes |

## 13. Compliance and Documentation

### 13.1 Design Documentation
Required thermal design documentation:
- Thermal simulation results showing junction temperatures
- Power map used for simulation
- Material properties table
- Cooling solution specification
- Thermal test plan and results

### 13.2 Qualification
Thermal qualification shall demonstrate:
- Junction temperatures remain within specifications across operating conditions
- Thermal resistance meets specifications
- No thermal runaway or excessive temperature gradients
- Reliability requirements met across temperature range

---

**© 2025 SmileStory Inc. / WIA**
弘益人間 (홍익인간) · Benefit All Humanity
