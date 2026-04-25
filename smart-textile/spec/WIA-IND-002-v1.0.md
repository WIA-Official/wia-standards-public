# WIA-IND-002: Smart Textile Standard - Technical Specification v1.0

> **Standard ID:** WIA-IND-002
> **Version:** 1.0.0
> **Status:** Active
> **Category:** IND / Industrial
> **Color:** Indigo (#6366F1)
> **Published:** 2025-01-15
> **Last Updated:** 2025-01-15

---

## Document Information

**Authors:**
- WIA Industrial Research Group
- Smart Materials Division
- Textile Engineering Committee

**Contributors:**
- University Textile Research Labs
- Healthcare Technology Experts
- Sports Science Institutes
- Military Research Organizations

**弘益人間 (Benefit All Humanity)**

This standard is developed to benefit all humanity by revolutionizing textile technology, improving health monitoring, enhancing comfort, and enabling new applications that improve quality of life.

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Conductive Fiber Technologies](#5-conductive-fiber-technologies)
6. [Sensor-Embedded Textiles](#6-sensor-embedded-textiles)
7. [Temperature Regulation Systems](#7-temperature-regulation-systems)
8. [E-Textile Architecture](#8-e-textile-architecture)
9. [Health Monitoring Systems](#9-health-monitoring-systems)
10. [Energy Harvesting](#10-energy-harvesting)
11. [Washability and Durability](#11-washability-and-durability)
12. [Wireless Communication](#12-wireless-communication)
13. [Manufacturing Processes](#13-manufacturing-processes)
14. [Testing and Certification](#14-testing-and-certification)
15. [Safety and Compliance](#15-safety-and-compliance)

---

## 1. Introduction

### 1.1 Purpose

The WIA-IND-002 standard provides comprehensive specifications for smart textile technology, integrating electronics, sensors, and advanced materials into conventional textiles. This standard addresses the growing need for wearable technology that is comfortable, washable, durable, and functionally integrated into everyday clothing.

### 1.2 Philosophy: 弘益人間 (Benefit All Humanity)

Smart textiles represent a convergence of materials science, electronics, and textile engineering to serve humanity:

- **Healthcare**: Continuous, non-invasive health monitoring for early disease detection
- **Comfort**: Adaptive clothing that responds to environmental conditions
- **Safety**: Protection for workers, military personnel, and first responders
- **Accessibility**: Assistive technology for elderly and disabled populations
- **Sustainability**: Longer-lasting, multi-functional garments reducing waste

### 1.3 Background

Traditional textiles serve basic functions: protection, comfort, and aesthetics. Smart textiles expand these capabilities by adding:
- **Sensing**: Detecting physical, chemical, and biological signals
- **Actuation**: Responding to stimuli (heating, cooling, color change)
- **Communication**: Transmitting data wirelessly
- **Energy**: Harvesting and storing energy from the environment

### 1.4 Target Applications

- Medical monitoring and patient care
- Sports and fitness tracking
- Military and first responder equipment
- Fashion and consumer electronics
- Industrial worker safety
- Elderly care and assisted living

---

## 2. Scope

### 2.1 Included in Standard

This standard covers:

1. **Conductive Fiber Materials**: Silver, copper, graphene, carbon nanotubes, conductive polymers
2. **Sensor Technologies**: ECG, EMG, temperature, pressure, strain, humidity
3. **Temperature Regulation**: PCM, thermoelectric, active heating/cooling
4. **E-Textile Circuits**: Conductive pathways, interconnects, component integration
5. **Health Monitoring**: Vital sign measurement and analysis
6. **Energy Harvesting**: Piezoelectric, thermoelectric, photovoltaic
7. **Communication Protocols**: BLE, NFC, RFID, wireless data transmission
8. **Manufacturing Standards**: Weaving, knitting, printing, coating
9. **Durability Testing**: Wash cycles, mechanical stress, environmental exposure
10. **Safety Standards**: Electrical safety, biocompatibility, data privacy

### 2.2 Excluded from Standard

- Pure fashion design (aesthetics without functional electronics)
- Non-wearable textile sensors (industrial process monitoring)
- Medical devices requiring invasive procedures
- Textile-based displays and user interfaces (covered in WIA-IND-003)

---

## 3. Normative References

### 3.1 International Standards

- **ISO 6330**: Textiles - Domestic washing and drying procedures
- **ISO 10993**: Biological evaluation of medical devices
- **ISO 139**: Textiles - Standard atmospheres for conditioning and testing
- **IEC 61340-4-9**: Electrostatic properties of materials
- **IEC 60601-1**: Medical electrical equipment safety
- **ASTM D5034**: Breaking strength and elongation of textile fabrics
- **AATCC 61**: Colorfastness to laundering
- **EN 13795**: Surgical drapes and gowns

### 3.2 Related WIA Standards

- **WIA-HEALTH-001**: Personal Health Monitoring
- **WIA-ENERGY-005**: Energy Harvesting Systems
- **WIA-COMMS-012**: Short-Range Wireless Communication
- **WIA-SAFETY-008**: Wearable Device Safety

---

## 4. Terms and Definitions

### 4.1 General Terms

**Smart Textile (스마트 섬유)**
A textile structure that can sense, react to, and adapt to environmental conditions or stimuli from mechanical, thermal, chemical, electrical, or magnetic sources.

**E-Textile (전자 섬유)**
A textile with integrated electronic functionality, including sensors, actuators, processors, and communication devices.

**Conductive Fiber (전도성 섬유)**
A fiber or yarn with electrical conductivity achieved through metallic coating, inherent conductive materials, or conductive polymer integration.

**Wearability (착용성)**
The degree to which a smart textile maintains comfort, flexibility, and usability when worn on the body.

### 4.2 Conductive Materials

**Silver-Plated Nylon (은 도금 나일론)**
Nylon fibers coated with metallic silver, providing high conductivity (10³-10⁵ S/m) with good flexibility.

**Graphene Fiber (그래핀 섬유)**
Carbon-based fiber with exceptional strength and conductivity, suitable for advanced sensor applications.

**Carbon Nanotube (CNT) (탄소 나노튜브)**
Cylindrical carbon molecules with high conductivity and mechanical strength, used in composite fibers.

### 4.3 Sensor Terms

**Gauge Factor (GF)**
The ratio of relative change in electrical resistance to mechanical strain: GF = (ΔR/R₀)/ε

**Sensitivity (민감도)**
The minimum detectable change in the measured parameter.

**Signal-to-Noise Ratio (SNR)**
The ratio of signal power to noise power, typically expressed in decibels (dB).

### 4.4 Temperature Regulation

**Phase Change Material (PCM) (상변화 물질)**
Materials that absorb or release latent heat during phase transitions (solid-liquid, liquid-gas).

**Thermoelectric Effect (열전 효과)**
The direct conversion of temperature differences to electric voltage (Seebeck effect) or vice versa (Peltier effect).

**Moisture Vapor Transmission Rate (MVTR)**
The rate at which water vapor passes through a fabric, measured in g/m²/day.

### 4.5 Energy Harvesting

**Piezoelectric Effect (압전 효과)**
Generation of electric charge in response to applied mechanical stress.

**Triboelectric Effect (마찰전기 효과)**
Electric charge generation through contact and separation of dissimilar materials.

**Power Density (전력 밀도)**
Power output per unit area, typically measured in μW/cm² or mW/m².

---

## 5. Conductive Fiber Technologies

### 5.1 Metallic Fibers

#### 5.1.1 Silver-Based Fibers

**Properties:**
- Conductivity: 1×10³ to 1×10⁵ S/m
- Flexibility: Excellent (can be woven and knitted)
- Washability: Good (50-100 cycles with proper encapsulation)
- Biocompatibility: Good (antimicrobial properties)
- Cost: High (4-10 USD/meter for yarn)

**Manufacturing Methods:**
1. **Electroless Plating**: Chemical deposition of silver onto nylon or polyester
2. **Physical Vapor Deposition (PVD)**: Vacuum deposition of silver coating
3. **Sputtering**: High-energy bombardment to deposit silver layer

**Conductivity Calculation:**

```
σ_effective = σ_Ag × f_vol × F_align × F_contact
```

Where:
- σ_Ag = Bulk silver conductivity (6.3×10⁷ S/m)
- f_vol = Volume fraction of silver (0.001-0.1)
- F_align = Alignment factor (0.3-1.0)
- F_contact = Inter-fiber contact factor (0.1-0.5)

**Typical Values:**
- Silver-plated nylon (Ag/PA66): 1×10⁴ S/m
- Silver-plated polyester (Ag/PET): 5×10³ S/m

#### 5.1.2 Copper Fibers

**Properties:**
- Conductivity: 1×10⁶ to 1×10⁷ S/m
- Flexibility: Poor to fair
- Washability: Fair (oxidation concerns)
- Cost: Low (0.5-2 USD/meter)

**Applications:**
- Heating elements in jackets and gloves
- EMI shielding fabrics
- Power transmission in e-textiles

#### 5.1.3 Stainless Steel Fibers

**Properties:**
- Conductivity: 1×10⁶ S/m
- Flexibility: Fair
- Washability: Excellent (>200 cycles)
- Durability: Excellent
- Cost: Low (0.3-1 USD/meter)

**Applications:**
- High-durability applications
- Cut-resistant fabrics
- EMI shielding

### 5.2 Carbon-Based Fibers

#### 5.2.1 Graphene Fibers

**Properties:**
- Conductivity: 1×10⁴ to 1×10⁵ S/m
- Flexibility: Excellent
- Mechanical Strength: Very high (>1 GPa tensile strength)
- Washability: Excellent
- Cost: Very High (>20 USD/meter)

**Production Methods:**
1. **Wet Spinning**: Graphene oxide dispersion → reduction → fiber
2. **Chemical Vapor Deposition (CVD)**: Growth on substrate → transfer
3. **Electrospinning**: Polymer/graphene composite → carbonization

**Applications:**
- High-end sensors requiring flexibility
- Structural health monitoring
- Advanced athletic wear

#### 5.2.2 Carbon Nanotube (CNT) Fibers

**Properties:**
- Conductivity: 1×10⁴ to 1×10⁶ S/m (depending on alignment)
- Flexibility: Excellent
- Strength: Exceptional (>10 GPa theoretical)
- Cost: High (10-30 USD/meter)

**CNT Types:**
- Single-Walled CNT (SWCNT): Higher conductivity, more expensive
- Multi-Walled CNT (MWCNT): Lower cost, good mechanical properties

**Yarn Production:**
1. **Direct Spinning**: From CNT aerogel or forest
2. **Wet Spinning**: CNT dispersion → coagulation
3. **Composite Spinning**: CNT mixed with polymer matrix

### 5.3 Conductive Polymers

#### 5.3.1 PEDOT:PSS

**Properties:**
- Conductivity: 1-10³ S/m (depending on formulation)
- Flexibility: Excellent
- Transparency: Can be optically transparent
- Processability: Solution-based coating/printing
- Stability: Moderate (degradation in water)

**Applications:**
- Flexible electrodes
- Transparent conductors
- Sensor coatings

#### 5.3.2 Polyaniline (PANI)

**Properties:**
- Conductivity: 1-100 S/m
- Environmental Sensitivity: Changes with pH, humidity
- Cost: Low

**Applications:**
- Chemical sensors
- pH-responsive textiles
- Corrosion protection

### 5.4 Conductive Inks and Coatings

#### 5.4.1 Silver Nanoparticle Inks

**Formulation:**
- Silver content: 20-60% by weight
- Particle size: 20-100 nm
- Sintering temperature: 120-200°C
- Sheet resistance: 0.1-1 Ω/sq (after sintering)

**Printing Methods:**
- Screen printing: Line width >50 μm
- Inkjet printing: Line width 20-100 μm
- Spray coating: Large area coverage

#### 5.4.2 Carbon-Based Inks

**Types:**
- Graphene ink: Conductivity 10-100 S/m
- Carbon black ink: Conductivity 1-10 S/m
- CNT ink: Conductivity 100-1000 S/m

**Advantages:**
- Lower cost than silver
- Better flexibility
- No oxidation issues

### 5.5 Fiber Selection Criteria

| Application | Recommended Fiber | Reason |
|-------------|------------------|---------|
| ECG Electrodes | Silver-plated nylon | High conductivity, biocompatibility |
| Strain Sensors | CNT/polymer composite | High gauge factor, flexibility |
| Heating Elements | Stainless steel or copper | Low cost, durability |
| Data Lines | Silver-plated yarn | Signal integrity, flexibility |
| EMI Shielding | Stainless steel | Durability, effectiveness |
| Chemical Sensors | PEDOT:PSS or PANI | Environmental sensitivity |

---

## 6. Sensor-Embedded Textiles

### 6.1 Physiological Sensors

#### 6.1.1 ECG (Electrocardiogram) Sensors

**Principle:**
Detection of electrical signals from cardiac activity through skin-electrode interface.

**Electrode Requirements:**
- Contact area: 5-20 cm²
- Skin-electrode impedance: <10 kΩ at 10 Hz
- Signal frequency: 0.05-150 Hz
- Amplitude: 0.5-4 mV (peak-to-peak)

**Textile Electrode Design:**

```
Electrode Structure:
[Skin] ← [Conductive layer] ← [Adhesive/gel layer] ← [Backing fabric]
```

**Materials:**
- Primary conductor: Silver/silver-chloride coating
- Backing: Polyester or polyamide knit
- Gel layer: Hydrogel or conductive polymer (optional)

**Performance Specifications:**
- Sampling rate: ≥250 Hz (preferably 500-1000 Hz)
- Resolution: 12-24 bits ADC
- Common mode rejection ratio (CMRR): >80 dB
- Input impedance: >10 MΩ

**Lead Configurations:**
- Single-lead: Simple heart rate monitoring
- 3-lead: Basic rhythm analysis
- 12-lead: Comprehensive diagnostic ECG (requires chest strap + limb electrodes)

#### 6.1.2 EMG (Electromyogram) Sensors

**Principle:**
Detection of electrical signals from muscle activation.

**Signal Characteristics:**
- Frequency range: 10-500 Hz
- Amplitude: 50 μV - 5 mV (surface EMG)
- Electrode spacing: 1-2 cm for local muscles

**Applications:**
- Muscle activity monitoring during exercise
- Rehabilitation therapy feedback
- Prosthetic control
- Fatigue detection

**Textile Implementation:**
- Dry electrodes: Silver-plated knit fabric
- Electrode size: 1-2 cm diameter
- Inter-electrode distance: 2 cm (bipolar configuration)

#### 6.1.3 Temperature Sensors

**Sensor Types:**

**A. Thermistor-Embedded Textiles**
- NTC thermistors: Negative temperature coefficient
- Resistance range: 1-100 kΩ at 25°C
- Accuracy: ±0.1-0.5°C
- Response time: 1-10 seconds

**B. Thermocouple-Based Sensors**
- Type K (Chromel-Alumel): -200 to 1350°C
- Type T (Copper-Constantan): -200 to 400°C
- Sensitivity: ~40 μV/°C

**C. Conductive Polymer Sensors**
- Resistance change with temperature
- TCR (Temperature Coefficient of Resistance): 1000-5000 ppm/°C

**Integration Methods:**
1. Embedded thermistors in fabric pockets
2. Printed conductive tracks with temperature-sensitive materials
3. Woven thermocouple junctions

**Calibration:**
```
R(T) = R₀ × exp[β(1/T - 1/T₀)]
```
Where:
- R(T) = Resistance at temperature T (K)
- R₀ = Resistance at reference temperature T₀
- β = Material constant (typically 3000-4000 K)

#### 6.1.4 Respiration Sensors

**Principle:**
Detect chest expansion/contraction during breathing using strain sensors.

**Sensor Design:**

**A. Resistive Strain Sensors**
```
ΔR/R₀ = GF × ε
```
Where:
- GF = Gauge factor (10-100 for textile sensors)
- ε = Strain (0.5-5% during breathing)

**B. Capacitive Sensors**
```
ΔC/C₀ = ε_strain × geometry_factor
```

**C. Piezoelectric Sensors**
- Voltage output proportional to strain rate
- Sensitive to breathing dynamics

**Placement:**
- Chest band: Thoracic breathing
- Abdominal band: Diaphragmatic breathing
- Dual placement: Total respiratory volume

**Signal Processing:**
- Breathing rate: 10-30 breaths/min (normal adult)
- Signal filtering: 0.1-1 Hz bandpass
- Pattern analysis: Apnea detection, irregular breathing

#### 6.1.5 Blood Oxygen (SpO₂) Sensors

**Principle:**
Photoplethysmography (PPG) using red and infrared light to measure oxygen saturation.

**Optical System:**
- Red LED: 660 nm wavelength
- IR LED: 940 nm wavelength
- Photodetector: Silicon photodiode

**Textile Integration Challenges:**
- Stable optical coupling to skin
- Ambient light rejection
- Motion artifact reduction

**Solutions:**
1. Flexible optical windows in fabric
2. Pressure-controlled contact using elastic bands
3. Accelerometer-based motion compensation

**Accuracy:**
- SpO₂ range: 70-100%
- Accuracy: ±2% (90-100% range)
- Heart rate accuracy: ±2 bpm

### 6.2 Physical Sensors

#### 6.2.1 Strain/Stretch Sensors

**Working Principle:**
Resistance or capacitance change with mechanical deformation.

**Sensor Types:**

**A. Resistive Strain Sensors**

Materials:
- CNT/polymer composites: GF = 1-50
- Graphene/polymer composites: GF = 10-100
- Silver nanowire networks: GF = 2-20

**Strain Sensitivity:**
```
GF = (ΔR/R₀) / ε
```

**Hysteresis:**
- Target: <5% over 1000 cycles
- Factors: Viscoelastic recovery, fiber slippage

**B. Capacitive Strain Sensors**

Structure:
```
[Top electrode] ← [Dielectric elastomer] ← [Bottom electrode]
```

Capacitance change:
```
ΔC/C₀ = ε × (2 + ν)
```
Where:
- ε = Applied strain
- ν = Poisson's ratio (~0.5 for elastomers)

**Applications:**
- Joint angle measurement (knees, elbows)
- Posture monitoring
- Gait analysis
- Breathing pattern detection

#### 6.2.2 Pressure Sensors

**Sensor Technologies:**

**A. Resistive Pressure Sensors**
- Piezoresistive materials: Carbon black, CNT, graphene
- Pressure range: 1-500 kPa
- Sensitivity: 0.1-10 kPa⁻¹

**B. Capacitive Pressure Sensors**
- Dielectric foam or elastomer
- Pressure range: 0.1-100 kPa
- Higher linearity than resistive

**C. Piezoelectric Pressure Sensors**
- PVDF (polyvinylidene fluoride) films
- Dynamic pressure measurement
- Self-powered (no bias voltage needed)

**Textile Integration:**
- Sock insoles: Gait analysis, diabetic foot monitoring
- Gloves: Grip force measurement
- Seating: Pressure ulcer prevention
- Mattresses: Sleep position tracking

**Pressure Mapping:**
- Sensor array: 8×8 to 32×32 taxels
- Spatial resolution: 5-10 mm
- Temporal resolution: 10-100 Hz

**Sensitivity Calculation:**
```
S = (ΔR/R₀) / ΔP
```
Where:
- ΔR/R₀ = Relative resistance change
- ΔP = Applied pressure (kPa)
- Typical S: 0.01-1 kPa⁻¹

#### 6.2.3 Humidity/Sweat Sensors

**Principle:**
Electrical property changes (resistance, capacitance) with moisture absorption.

**Sensor Materials:**
- Hygroscopic polymers: Polyimide, polyaniline
- Metal oxides: SnO₂, ZnO
- Carbon-based: Graphene oxide, CNT

**Sweat Rate Measurement:**
```
Sweat_rate (mg/cm²/hr) = ΔRH × calibration_factor
```

**Applications:**
- Hydration monitoring for athletes
- Thermal stress assessment
- Sleep quality analysis
- Wound moisture management

**Performance:**
- RH range: 20-95%
- Response time: <30 seconds
- Accuracy: ±3-5% RH

### 6.3 Chemical and Biosensors

#### 6.3.1 pH Sensors

**Materials:**
- Conductive polymers (PANI, polypyrrole)
- Metal oxide films (IrO₂, RuO₂)

**Response:**
```
E(pH) = E₀ + (RT/nF) × ln([H⁺])
     ≈ E₀ + 59.16 mV × pH (at 25°C)
```

**Applications:**
- Sweat pH monitoring (normal: 4.5-7.0)
- Wound healing assessment
- Skin condition monitoring

#### 6.3.2 Lactate Sensors

**Principle:**
Enzymatic detection (lactate oxidase) with electrochemical transduction.

**Reaction:**
```
Lactate + O₂ →(LOx)→ Pyruvate + H₂O₂
H₂O₂ → 2H⁺ + O₂ + 2e⁻ (at electrode)
```

**Applications:**
- Athletic performance monitoring
- Metabolic disorder detection
- Sepsis early warning

**Performance:**
- Detection range: 0-20 mM (sweat), 0-100 mM (blood through skin)
- Sensitivity: >10 μA/mM
- Response time: <1 minute

#### 6.3.3 Glucose Sensors

**Technologies:**
- Enzymatic (glucose oxidase)
- Non-enzymatic (metal nanoparticles, carbon materials)

**Challenges in Textiles:**
- Enzyme stability during washing
- Interference from sweat composition
- Calibration drift

**Applications:**
- Diabetes management
- Athletic nutrition monitoring

### 6.4 Environmental Sensors

#### 6.4.1 UV Sensors

**Photodetectors:**
- Silicon photodiodes with UV filter
- ZnO, TiO₂ nanostructures
- Organic photodetectors

**UV Index Calculation:**
```
UV_index = (E_erythema × 40 mW/m²) / (25 mW/m²)
```

**Integration:**
- Shoulder patches
- Hat brims
- Wristbands

**Applications:**
- Sun exposure monitoring
- Skin cancer prevention
- Outdoor worker safety

#### 6.4.2 Air Quality Sensors

**Pollutant Detection:**
- CO: Electrochemical sensors
- NO₂: Metal oxide sensors (SnO₂)
- VOCs: MEMS gas sensors
- PM2.5/PM10: Optical particle counters

**Miniaturization Challenges:**
- Power consumption (>50 mW for some sensors)
- Sensor size and weight
- Integration complexity

**Applications:**
- Urban air quality monitoring
- Occupational exposure assessment
- Asthma trigger detection

---

## 7. Temperature Regulation Systems

### 7.1 Phase Change Materials (PCM)

#### 7.1.1 PCM Types and Properties

**Organic PCMs:**

**A. Paraffins**
- Melting point range: 20-70°C
- Latent heat: 150-250 J/g
- Advantages: Non-corrosive, stable, recyclable
- Disadvantages: Low thermal conductivity, flammable

**B. Fatty Acids and Esters**
- Melting point: 25-65°C
- Latent heat: 120-200 J/g
- Biodegradable and renewable

**Inorganic PCMs:**

**A. Salt Hydrates**
- Examples: Glauber's salt (Na₂SO₄·10H₂O), Calcium chloride hexahydrate
- Melting point: 8-58°C
- Latent heat: 150-250 J/g
- Advantages: Higher thermal conductivity, lower cost
- Disadvantages: Corrosive, phase separation, supercooling

**B. Metallic PCMs**
- Low-melting alloys
- High thermal conductivity
- Expensive, heavy

#### 7.1.2 PCM Encapsulation

**Microencapsulation:**
- Particle size: 1-100 μm
- Shell materials: Melamine-formaldehyde, polyurethane, silica
- Core: PCM (paraffin, salt hydrate)

**Encapsulation Methods:**
1. **Coacervation**: Phase separation in polymer solution
2. **Interfacial Polymerization**: Shell formation at droplet interface
3. **Spray Drying**: Droplet solidification in hot air

**Macroencapsulation:**
- Larger containers: pouches, panels (>1 cm)
- Direct integration into garment pockets or panels

#### 7.1.3 PCM Integration into Textiles

**Methods:**

**A. Coating**
- Direct coating of microencapsulated PCM onto fabric
- Coating thickness: 50-200 μm
- PCM loading: 10-30% by weight

**B. Lamination**
- PCM layer between fabric layers
- Higher PCM content: 30-60% by weight

**C. Fiber Spinning**
- PCM incorporated during fiber production
- Core-sheath fibers: PCM core, polymer sheath
- Durability: Excellent (PCM protected inside fiber)

**Thermal Performance:**

**Energy Storage:**
```
Q = m × (c_p × ΔT + L_f)
```
Where:
- m = Mass of PCM (g)
- c_p = Specific heat capacity (J/g·K)
- ΔT = Temperature change
- L_f = Latent heat of fusion (J/g)

**Example Calculation:**
- PCM: Paraffin, melting point 28°C, L_f = 200 J/g
- PCM loading: 20% in 200 g garment → 40 g PCM
- Energy storage: Q = 40 g × 200 J/g = 8000 J = 8 kJ
- Thermal buffering: ~30-60 minutes at moderate activity

#### 7.1.4 Applications

- **Summer Clothing**: PCM melting at 28-32°C for cooling
- **Winter Clothing**: PCM melting at 20-25°C for thermal storage
- **Protective Gear**: Firefighters, industrial workers
- **Medical**: Temperature-controlled therapy garments

### 7.2 Active Heating Systems

#### 7.2.1 Resistive Heating

**Principle:**
Joule heating through current flow in conductive fibers.

**Power Calculation:**
```
P = V² / R = I² × R
```

**Heat Generation:**
```
Q = I² × R × t
```

**Design Parameters:**

**Heating Element Materials:**
- Stainless steel fiber: Resistance 10-100 Ω/m
- Carbon fiber: Resistance 50-500 Ω/m
- Silver-coated fiber: Resistance 1-10 Ω/m

**Power Density:**
- Target: 100-500 W/m² for comfort heating
- Maximum: 1000-2000 W/m² for rapid heating

**Voltage:**
- Low voltage DC: 3.7V (Li-ion), 5V (USB), 12V (automotive)
- Safety: <50V DC (no shock hazard)

**Current Limiting:**
- Overcurrent protection
- Thermal cutoff switches (>50°C)
- PTC (Positive Temperature Coefficient) materials for self-regulation

**Example Design:**
```
Target: 200 W/m² heating
Area: 0.1 m² (chest panel)
Power: 20 W
Voltage: 5V (USB power bank)
Current: I = P/V = 20W/5V = 4A
Resistance: R = V²/P = 25Ω / 5² = 1.25Ω
```

**Applications:**
- Heated jackets and gloves
- Motorcycle gear
- Outdoor sports apparel
- Medical thermotherapy

#### 7.2.2 Thermoelectric (Peltier) Heating/Cooling

**Principle:**
Peltier effect: Heat transfer through electrical current in semiconductor junctions.

**Peltier Device:**
- Thin-film thermoelectric modules
- Thickness: 1-5 mm
- COP (Coefficient of Performance): 0.3-0.6 for cooling, >1 for heating

**Heat Pumping:**
```
Q_c = α × T_c × I - 0.5 × I² × R - K × ΔT
```
Where:
- α = Seebeck coefficient
- T_c = Cold side temperature
- I = Current
- R = Electrical resistance
- K = Thermal conductance
- ΔT = Temperature difference

**Advantages:**
- Reversible (heating or cooling)
- Precise temperature control
- Solid-state (no moving parts)

**Disadvantages:**
- High power consumption (5-10 W per module)
- Requires heat sink on hot side
- Lower efficiency than PCM for buffering

**Applications:**
- Climate-controlled suits for extreme environments
- Medical cooling vests
- Military and aerospace applications

### 7.3 Active Cooling Systems

#### 7.3.1 Evaporative Cooling

**Principle:**
Latent heat of vaporization removes heat from body.

**Cooling Power:**
```
Q = m_water × L_v
```
Where:
- m_water = Mass of evaporated water (kg/s)
- L_v = Latent heat of vaporization (2.43 MJ/kg at 30°C)

**Textile Design:**
- High moisture-wicking fabrics (polyester, nylon microfibers)
- MVTR: >10,000 g/m²/day
- Capillary action for sweat distribution

**Passive Evaporative Cooling:**
- Moisture-wicking layers
- Breathable membranes (PTFE, PU)

**Active Evaporative Cooling:**
- Water circulation systems
- Evaporation-enhanced fabrics

#### 7.3.2 Radiative Cooling

**Principle:**
Emission of thermal radiation to cool the body, especially effective in infrared transparent "atmospheric window" (8-13 μm).

**Radiative Cooling Power:**
```
P_rad = ε × σ × A × (T_body⁴ - T_sky⁴)
```
Where:
- ε = Emissivity in IR range
- σ = Stefan-Boltzmann constant (5.67×10⁻⁸ W/m²·K⁴)
- A = Surface area
- T_body, T_sky = Body and sky temperatures (K)

**Nanophotonic Textiles:**
- High emissivity in 8-13 μm range
- Low solar absorption (<5%)
- Materials: PE fibers, nanoporous polymers

**Cooling Enhancement:**
- Up to 5°C below ambient in direct sunlight
- No power required (passive cooling)

### 7.4 Breathability and Moisture Management

#### 7.4.1 Moisture Vapor Transmission

**MVTR Measurement:**
- Cup method (ASTM E96)
- Dynamic method (Sweating guarded hotplate)

**Target MVTR Values:**
- Athletic wear: >10,000 g/m²/day
- Outdoor jackets: 5,000-20,000 g/m²/day
- Protective clothing: >5,000 g/m²/day

#### 7.4.2 Moisture-Wicking

**Capillary Action:**
```
h = (2 × γ × cos θ) / (ρ × g × r)
```
Where:
- h = Height of liquid rise
- γ = Surface tension
- θ = Contact angle
- ρ = Liquid density
- g = Gravity
- r = Capillary radius

**Wicking Rate:**
- Fast-wicking: >5 cm in 10 seconds
- Materials: Polyester microfibers, nylon, treated cotton

---

## 8. E-Textile Architecture

### 8.1 Circuit Design

#### 8.1.1 Conductive Pathways

**Routing Methods:**

**A. Woven Conductors**
- Conductive yarns in warp or weft
- Insulation: Non-conductive yarns or coatings
- Resistance: 0.1-10 Ω/cm

**B. Embroidered Circuits**
- CNC embroidery machines
- Line width: 0.5-2 mm
- Multilayer capability

**C. Printed Circuits**
- Screen printing, inkjet printing
- Line width: 50-500 μm
- Sheet resistance: 0.1-10 Ω/sq

**Design Considerations:**
- Crosstalk: Spacing >2 mm for low-speed signals
- Impedance matching: 50 Ω for RF, 75 Ω for video
- Current capacity: 0.1-1 A/mm² (depending on conductor thickness)

#### 8.1.2 Component Integration

**Component Types:**
- Microcontrollers (MCU)
- Sensors and transducers
- LEDs and displays
- Power management ICs
- Wireless modules (BLE, NFC)

**Attachment Methods:**

**A. Snap Fasteners**
- Removable components
- Wash-friendly (remove before washing)
- Contact resistance: <0.1 Ω

**B. Conductive Adhesives**
- Permanent attachment
- Silver epoxy, conductive tape
- Flexible, stretchable adhesives

**C. Soldering**
- Traditional soldering to conductive pads
- Low-temperature solder (<150°C) for heat-sensitive fabrics

**D. Anisotropic Conductive Film (ACF)**
- Z-axis conductivity only
- Fine pitch connections (<500 μm)

**Component Encapsulation:**
- Protective coatings: Silicone, polyurethane
- Rigid islands in flexible substrates
- IP ratings: IP65-IP68 for water resistance

### 8.2 Power Systems

#### 8.2.1 Battery Integration

**Battery Types:**

**A. Lithium Polymer (LiPo)**
- Energy density: 150-250 Wh/kg
- Voltage: 3.7V nominal (3.0-4.2V range)
- Form factors: Flat pouches, ideal for textiles
- Capacity: 100-5000 mAh typical

**B. Flexible Batteries**
- Thin-film batteries: 0.1-0.5 mm thick
- Energy density: 50-150 Wh/kg
- Flexible, bendable (>1000 cycles)

**C. Printed Batteries**
- Zinc-based chemistry
- Ultra-thin: <1 mm
- Low capacity: 1-50 mAh
- Disposable or limited recharge

**Battery Placement:**
- Pockets with Velcro or zipper
- Integrated pouches in lining
- Modular, removable for washing

**Safety:**
- Overcharge protection
- Thermal cutoff (>60°C)
- Short circuit protection
- Puncture-resistant enclosure

#### 8.2.2 Power Management

**DC-DC Converters:**
- Buck converters: Step down voltage (e.g., 5V → 3.3V)
- Boost converters: Step up voltage (e.g., 3.7V → 5V)
- Efficiency: >85%

**Voltage Regulation:**
- LDO (Low Dropout) regulators: Simple, low noise
- Switching regulators: Higher efficiency

**Power Budget:**
```
Total_Power = P_MCU + P_sensors + P_wireless + P_other
```

**Example:**
- MCU (sleep mode): 10 μA @ 3.3V = 33 μW
- MCU (active): 5 mA @ 3.3V = 16.5 mW
- BLE (advertising): 5 mA @ 3.3V = 16.5 mW
- BLE (connected): 10 mA @ 3.3V = 33 mW
- Sensors (ECG): 2 mA @ 3.3V = 6.6 mW
- Total (continuous monitoring): ~60 mW
- Battery life: 500 mAh × 3.7V / 60 mW = 30 hours

#### 8.2.3 Energy Harvesting Integration

**Harvested Power:**
- Piezoelectric: 1-100 μW (walking, running)
- Thermoelectric: 10-500 μW (body heat)
- Solar (indoor): 10-100 μW/cm²
- Solar (outdoor): 100-1000 μW/cm²

**Power Conditioning:**
- Rectification (AC to DC)
- Maximum Power Point Tracking (MPPT)
- Energy storage (supercapacitors, batteries)

**Hybrid Power:**
- Primary battery + energy harvesting
- Extends battery life 2-10x

### 8.3 Data Processing

#### 8.3.1 Microcontrollers

**Requirements:**
- Low power: Sleep mode <10 μA
- Sufficient I/O: ADC, I2C, SPI, UART
- Processing: ARM Cortex-M0/M4 (16-100 MHz)
- Memory: 32-512 KB Flash, 8-64 KB RAM

**Popular MCUs for Wearables:**
- Nordic nRF52832/52840: BLE + ARM Cortex-M4
- STM32L series: Ultra-low power
- ESP32: WiFi + BLE, higher power

#### 8.3.2 Signal Processing

**Analog Front-End (AFE):**
- Amplification: 100-10,000x for bioelectric signals
- Filtering: Bandpass (0.05-150 Hz for ECG)
- ADC: 12-24 bit resolution, 100-1000 Hz sampling

**Digital Filtering:**
- Finite Impulse Response (FIR) filters
- Infinite Impulse Response (IIR) filters
- Adaptive filters for noise cancellation

**Feature Extraction:**
- Heart rate from ECG: R-peak detection
- Respiration rate: Peak counting in strain sensor
- Activity classification: Accelerometer pattern recognition

### 8.4 Communication Interfaces

**Wired:**
- I2C: Sensor communication (100-400 kHz)
- SPI: High-speed peripherals (1-10 MHz)
- UART: Serial debugging, GPS

**Wireless:**
- BLE: Primary wireless interface
- NFC: Configuration, pairing
- WiFi: High data rate (ESP32)
- LoRa: Long-range, low power

---

## 9. Health Monitoring Systems

### 9.1 Cardiac Monitoring

#### 9.1.1 Heart Rate Measurement

**Methods:**

**A. ECG-Based**
- R-peak detection in QRS complex
- Accuracy: ±1-2 bpm
- Gold standard for arrhythmia detection

**B. PPG-Based (Photoplethysmography)**
- Optical heart rate sensing
- Accuracy: ±2-5 bpm
- Affected by motion artifacts

**Heart Rate Calculation:**
```
HR (bpm) = 60 / RR_interval (seconds)
```

**Heart Rate Variability (HRV):**
```
SDNN = √(Σ(RR_i - RR_mean)² / (N-1))
```
- Indicator of autonomic nervous system function
- Stress, fitness, recovery assessment

#### 9.1.2 Arrhythmia Detection

**Common Arrhythmias:**
- Atrial Fibrillation (AFib): Irregular RR intervals
- Premature Ventricular Contractions (PVC): Abnormal QRS morphology
- Bradycardia: HR <60 bpm
- Tachycardia: HR >100 bpm

**Detection Algorithms:**
- Template matching
- Machine learning classifiers (SVM, neural networks)
- Rule-based expert systems

**Accuracy:**
- Sensitivity: >90% for AFib detection
- Specificity: >95%

### 9.2 Respiration Monitoring

#### 9.2.1 Measurement Methods

**A. Thoracic Impedance**
- Electrical impedance changes with lung volume
- Z = V / I, ΔZ ∝ ΔLung_volume

**B. Strain-Based**
- Chest/abdominal circumference change
- ΔR/R or ΔC/C

**C. Accelerometer**
- Chest motion during breathing

**Respiration Rate:**
```
RR (breaths/min) = 60 / breathing_cycle_period (seconds)
```

**Normal Values:**
- Adults: 12-20 breaths/min
- Children: 20-30 breaths/min
- Infants: 30-60 breaths/min

#### 9.2.2 Respiratory Patterns

**Pattern Analysis:**
- Regular vs. irregular breathing
- Apnea detection (pause >10 seconds)
- Hyperventilation (RR >25 breaths/min)
- Cheyne-Stokes breathing (crescendo-decrescendo pattern)

**Applications:**
- Sleep apnea screening
- Respiratory disease monitoring (COPD, asthma)
- Stress and anxiety assessment

### 9.3 Temperature Monitoring

#### 9.3.1 Core vs. Skin Temperature

**Core Temperature:**
- Normal: 36.5-37.5°C
- Measurement: Ear, rectal, esophageal (invasive)
- Estimation from skin: Dual heat flux sensors

**Skin Temperature:**
- Normal: 30-34°C (varies by location)
- Direct measurement with textile thermistors

**Heat Flux Method:**
```
T_core = T_skin + (HF / h)
```
Where:
- HF = Heat flux (W/m²)
- h = Heat transfer coefficient

#### 9.3.2 Thermal Stress Assessment

**Heat Stress Indicators:**
- Physiological Strain Index (PSI)
- Wet Bulb Globe Temperature (WBGT)
- Core temperature >38°C: Heat exhaustion risk

**Cold Stress:**
- Skin temperature <30°C: Discomfort
- Core temperature <35°C: Hypothermia

### 9.4 Activity and Posture Monitoring

#### 9.4.1 Accelerometer-Based

**3-Axis Accelerometer:**
- Activity classification: Walking, running, cycling, sitting
- Fall detection: Sudden acceleration >3g
- Sleep tracking: Movement patterns

**Activity Intensity:**
```
Activity_counts = Σ|a(t) - a_mean| (over epoch)
```

**Energy Expenditure:**
```
METs = 1.0 + 0.0175 × Activity_counts
```

#### 9.4.2 Posture Classification

**Strain Sensor Array:**
- Joint angle estimation
- Slouching detection
- Ergonomic assessment

**Applications:**
- Workplace ergonomics
- Rehabilitation therapy
- Sports training

### 9.5 Clinical Applications

#### 9.5.1 Continuous Patient Monitoring

**Hospital Settings:**
- ICU patients: Continuous ECG, respiration, temperature
- Post-surgical monitoring
- Neonatal monitoring

**Home Care:**
- Chronic disease management (heart failure, COPD)
- Elderly care and fall prevention
- Remote patient monitoring (telemedicine)

#### 9.5.2 Disease-Specific Monitoring

**Cardiovascular:**
- Atrial fibrillation screening
- Heart failure decompensation
- Ischemia detection

**Respiratory:**
- COPD exacerbation
- Asthma attack prediction
- Sleep apnea severity

**Metabolic:**
- Diabetes (glucose, activity, stress)
- Obesity management

**Neurological:**
- Epilepsy (seizure detection)
- Parkinson's disease (tremor, gait)

---

## 10. Energy Harvesting

### 10.1 Piezoelectric Energy Harvesting

#### 10.1.1 Principles

**Piezoelectric Effect:**
Mechanical stress → Electric charge

**Materials:**
- PVDF (Polyvinylidene Fluoride): Flexible, d₃₃ = 20-30 pC/N
- PZT (Lead Zirconate Titanate): High performance, rigid, d₃₃ = 200-600 pC/N
- ZnO nanowires: Flexible, biocompatible

**Voltage Generation:**
```
V = g₃₃ × t × σ
```
Where:
- g₃₃ = Voltage coefficient (V·m/N)
- t = Thickness (m)
- σ = Stress (N/m²)

**Power Output:**
```
P = 0.5 × C × V² × f
```
Where:
- C = Capacitance (F)
- V = Voltage (V)
- f = Frequency (Hz)

#### 10.1.2 Textile Integration

**PVDF Fibers:**
- Woven or knitted into fabric
- Placement: High-strain locations (joints, chest)

**Hybrid Structures:**
- Textile substrate + PVDF film lamination
- Enhanced mechanical coupling

**Harvested Power:**
- Walking (1 Hz): 10-100 μW per cm²
- Running (2-3 Hz): 50-500 μW per cm²
- Arm movement: 5-50 μW per cm²

**Applications:**
- Shoe insoles: Up to 1-5 mW while walking
- Backpack straps: 100-500 μW
- Clothing (chest, shoulders): 50-200 μW

### 10.2 Thermoelectric Energy Harvesting

#### 10.2.1 Seebeck Effect

**Principle:**
Temperature gradient → Voltage

**Thermoelectric Voltage:**
```
V = α × ΔT
```
Where:
- α = Seebeck coefficient (μV/K)
- ΔT = Temperature difference (K)

**Power Output:**
```
P_max = (α² × ΔT²) / (4 × R_internal)
```

**Figure of Merit (ZT):**
```
ZT = (α² × σ × T) / κ
```
Where:
- σ = Electrical conductivity
- κ = Thermal conductivity
- T = Absolute temperature

#### 10.2.2 Textile Thermoelectric Generators (TEG)

**Materials:**
- Bi₂Te₃: Best room-temperature performance, ZT ≈ 1
- Conductive polymers: Flexible, low ZT ≈ 0.1-0.3
- Carbon nanotube composites: ZT ≈ 0.01-0.1

**Body Heat Harvesting:**
- Skin temperature: ~33°C
- Ambient temperature: 20-25°C
- ΔT: 8-13 K

**Power Density:**
- Rigid TEG: 10-50 μW/cm² (ΔT = 10 K)
- Flexible TEG: 1-10 μW/cm² (ΔT = 10 K)

**Challenges:**
- Thermal insulation (prevent heat loss to ambient)
- Mechanical flexibility vs. efficiency trade-off
- Cost and complexity

**Applications:**
- Wristbands: 100-500 μW
- Headbands: 50-200 μW
- Full garment: 1-5 mW (with good insulation)

### 10.3 Solar Energy Harvesting

#### 10.3.1 Flexible Solar Cells

**Technologies:**
- Thin-film silicon: 5-10% efficiency
- Organic photovoltaics (OPV): 8-15% efficiency
- Dye-sensitized solar cells (DSSC): 10-12% efficiency
- Perovskite solar cells: 15-20% efficiency (emerging)

**Textile Integration:**
- Laminated solar panels on fabric
- Solar cell fibers (experimental)
- Flexible modules sewn into garments

**Power Output:**
- Indoor lighting (200-500 lux): 10-50 μW/cm²
- Outdoor (full sun, 100,000 lux): 10-20 mW/cm²

**Applications:**
- Backpacks with solar panels: 5-10 W
- Jackets with shoulder/back panels: 1-5 W
- Hats with solar cells: 0.5-2 W

#### 10.3.2 Design Considerations

**Solar Panel Placement:**
- High exposure areas: Shoulders, back, top of hat
- Avoid shadowing from arms, head

**Efficiency Factors:**
- Angle to sun: Optimal at 90° incidence
- Temperature: Efficiency decreases ~0.5%/°C above 25°C
- Partial shading: Bypass diodes to prevent hotspots

### 10.4 Triboelectric Energy Harvesting

#### 10.4.1 Principle

**Triboelectric Effect:**
Contact-induced charge transfer + electrostatic induction

**Power Generation:**
- Contact-separation mode
- Sliding mode
- Single-electrode mode

**Materials:**
- Positive triboelectric: Nylon, polyester, silk
- Negative triboelectric: PTFE, silicone, Kapton

**Output:**
- Voltage: 10-1000 V (open circuit)
- Current: 0.1-100 μA (short circuit)
- Power: 0.1-10 μW/cm² (average)

#### 10.4.2 Textile Applications

**Fabric-Fabric Interaction:**
- Layered clothing with different materials
- Motion-activated during walking, arm swinging

**Shoe Insoles:**
- Triboelectric layers in sole
- Step-activated energy generation

**Power Output:**
- Walking: 10-100 μW per step
- Fabrics rubbing: 1-10 μW/cm²

**Challenges:**
- Intermittent power (requires energy storage)
- Environmental humidity effects
- Power conditioning complexity

### 10.5 Hybrid Energy Harvesting

**Combination Strategies:**
- Piezoelectric + Triboelectric: Mechanical energy
- Thermoelectric + Solar: Continuous + intermittent
- Multiple sources: Maximize total harvested power

**Power Management:**
- Energy storage: Supercapacitors, rechargeable batteries
- MPPT (Maximum Power Point Tracking)
- Voltage regulation and conditioning

**Total Harvested Power (Example):**
- Piezoelectric (walking): 500 μW
- Thermoelectric (body heat): 200 μW
- Solar (outdoor): 5000 μW (intermittent)
- Total average: ~1-2 mW
- Sufficient for: Low-power sensors, BLE, basic MCU

---

## 11. Washability and Durability

### 11.1 Wash Test Protocols

#### 11.1.1 Standard Procedures

**ISO 6330:**
- Domestic washing and drying
- Test procedures: A through F (varying temperature, agitation)
- Temperature: 30°C, 40°C, 60°C, 95°C

**IEC 61340-4-9:**
- Electrostatic properties after laundering
- Resistance measurement before/after cycles

**Test Cycle:**
```
Wash → Rinse → Spin → Dry → Cool → Measure
```

**Number of Cycles:**
- Consumer products: 50-100 cycles
- Medical/professional: 100-200 cycles
- Military/industrial: 200-500 cycles

#### 11.1.2 Performance Metrics

**Electrical Properties:**
- Resistance retention: Target >80% after 50 cycles
- Continuity: No open circuits
- Insulation: No short circuits

**Mechanical Properties:**
- Tensile strength: >90% retention
- Elongation: <10% change
- Abrasion resistance: Grade 3-4 (ISO 12947)

**Sensor Accuracy:**
- ECG: <5% signal degradation
- Strain sensors: GF change <10%
- Temperature: <0.5°C accuracy drift

### 11.2 Encapsulation and Protection

#### 11.2.1 Coating Materials

**Waterproof Coatings:**
- Polyurethane (PU): Flexible, breathable
- Silicone: Excellent water resistance, flexible
- Parylene: Conformal, thin (0.5-50 μm), biocompatible
- PDMS (Polydimethylsiloxane): Stretchable, hydrophobic

**Application Methods:**
- Dip coating
- Spray coating
- Vapor deposition (Parylene)

**Coating Thickness:**
- Thin: 1-10 μm (maintains flexibility)
- Medium: 10-100 μm (good protection)
- Thick: 100-500 μm (maximum protection, reduced flexibility)

#### 11.2.2 Component Encapsulation

**Electronics Encapsulation:**
- Potting compounds: Epoxy, silicone
- Overmolding: Thermoplastic injection molding
- Hermetic sealing: Metal or glass enclosures

**IP Ratings:**
- IP65: Dust-tight, water jets (machine washable with care)
- IP67: Dust-tight, immersion up to 1m (hand washable)
- IP68: Dust-tight, continuous immersion (fully washable)

**Connector Protection:**
- Sealed connectors with O-rings
- Magnetic pogo pins with waterproof coating
- Wireless charging (no exposed contacts)

### 11.3 Mechanical Durability

#### 11.3.1 Flex Testing

**Flex Cycles:**
- Light duty: 10,000 cycles
- Normal duty: 100,000 cycles
- Heavy duty: 1,000,000 cycles

**Flex Radius:**
- Tight bend: 5 mm radius
- Normal bend: 10-20 mm radius
- Loose bend: >50 mm radius

**Test Setup:**
- Cyclic bending machine
- Resistance monitoring during flexing
- Failure: >10% resistance increase or open circuit

#### 11.3.2 Abrasion Resistance

**Martindale Abrasion Test (ISO 12947):**
- Rubbing cycles until failure
- Grade 1: <5,000 cycles
- Grade 2: 5,000-15,000 cycles
- Grade 3: 15,000-40,000 cycles
- Grade 4: 40,000-100,000 cycles
- Grade 5: >100,000 cycles

**Protective Strategies:**
- Reinforced patches over conductive traces
- Embedded conductors (below surface)
- Abrasion-resistant top layer

### 11.4 Chemical Resistance

#### 11.4.1 Detergent Compatibility

**Common Detergents:**
- Anionic surfactants: Most common, generally compatible
- Cationic surfactants: Fabric softeners, may affect conductivity
- Enzymes: Can degrade natural fibers, avoid for silk sensors

**pH Range:**
- Neutral: pH 6-8 (safest for most materials)
- Alkaline: pH 9-11 (standard detergents)
- Acidic: pH 4-6 (wool and silk)

**Testing:**
- Immersion in detergent solution
- Temperature: 30-60°C
- Duration: 30-60 minutes per cycle

#### 11.4.2 Sweat and Body Fluid Resistance

**Sweat Composition:**
- pH: 4.5-7.0 (slightly acidic)
- Salts: NaCl, KCl, lactic acid
- Organic compounds: Urea, amino acids

**Accelerated Testing:**
- Artificial sweat solution (ISO 3160-2)
- Elevated temperature (37°C)
- Extended exposure (24-72 hours)

**Corrosion Resistance:**
- Silver: Tarnishing (Ag₂S formation)
- Copper: Oxidation (Cu₂O, CuO)
- Stainless steel: Excellent resistance
- Protective coatings: Prevent corrosion

### 11.5 Long-Term Stability

**Accelerated Aging:**
- Elevated temperature (60-80°C)
- Humidity (80-95% RH)
- Duration: 500-1000 hours
- Extrapolation: Estimate 5-10 year lifetime

**Storage Conditions:**
- Temperature: 15-25°C
- Humidity: 40-60% RH
- Avoid: Direct sunlight, extreme temperatures

**Shelf Life:**
- Conductive textiles: 2-5 years (properly stored)
- Sensors with enzymes: 6-12 months
- Batteries: 1-3 years (self-discharge)

---

## 12. Wireless Communication

### 12.1 Bluetooth Low Energy (BLE)

#### 12.1.1 BLE Basics

**Frequency:**
- 2.4 GHz ISM band
- 40 channels (37 data, 3 advertising)
- Channel spacing: 2 MHz

**Data Rate:**
- BLE 4.x: 1 Mbps
- BLE 5.0: Up to 2 Mbps (or 125/500 kbps for long range)

**Power Consumption:**
- Advertising: 5-15 mA (intermittent)
- Connected: 10-20 mA (active communication)
- Sleep: 1-10 μA

**Range:**
- Standard: 10-50 meters
- BLE 5.0 long range: Up to 200-400 meters (line of sight)

#### 12.1.2 BLE Profiles for Health

**Standard Profiles:**
- **Heart Rate Profile (HRP)**: Heart rate measurement
- **Health Thermometer Profile (HTP)**: Temperature data
- **Glucose Profile (GLP)**: Glucose measurements
- **Blood Pressure Profile (BLP)**: Blood pressure data
- **Continuous Glucose Monitoring (CGM)**: Real-time glucose

**Custom Services:**
- Multi-sensor data aggregation
- Raw signal streaming (ECG, EMG)
- Configuration and control

**Data Throughput:**
- MTU (Maximum Transmission Unit): 20-244 bytes
- Connection interval: 7.5-4000 ms
- Max throughput: ~100 kbps (BLE 4.2), ~200 kbps (BLE 5.0)

#### 12.1.3 Antenna Integration

**Antenna Types:**
- Chip antenna: Small (3×3 mm), moderate performance
- PCB antenna: Low cost, good performance
- External antenna: Best performance, requires space
- Textile antenna: Conductive thread, integrated in fabric

**Textile Antenna Design:**

**A. Embroidered Antenna**
- Conductive thread (silver-plated)
- Dimensions: Quarter-wave (~31 mm at 2.4 GHz)
- Efficiency: 30-60% (lower than rigid antennas)

**B. Screen-Printed Antenna**
- Conductive ink on fabric
- Better efficiency: 50-70%
- Thinner profile

**Antenna Placement:**
- Away from body (10-20 mm gap) to reduce absorption
- Ground plane: Conductive fabric layer
- Size: λ/4 to λ/2 (31-62 mm at 2.4 GHz)

### 12.2 Near Field Communication (NFC)

#### 12.2.1 NFC Basics

**Frequency:**
- 13.56 MHz (ISM band)

**Operating Modes:**
- Reader/Writer: Read NFC tags
- Peer-to-Peer: Exchange data between devices
- Card Emulation: Act as contactless card

**Range:**
- Typical: 1-4 cm
- Maximum: ~10 cm

**Data Rate:**
- 106, 212, 424, 848 kbps

#### 12.2.2 NFC in Smart Textiles

**Applications:**
- Device pairing (tap to pair with smartphone)
- Product authentication (anti-counterfeiting)
- User identification (access control)
- Quick configuration (tap to program settings)

**NFC Antenna:**
- Coil antenna: Multiple turns of conductive thread or wire
- Inductance: 1-10 μH
- Dimensions: 30×50 mm to 50×80 mm

**Textile NFC Antenna:**
- Embroidered or woven conductive traces
- Ferrite sheet to improve coupling and reduce body absorption
- Encapsulation for wash resistance

### 12.3 WiFi and Cellular

#### 12.3.1 WiFi (802.11)

**Advantages:**
- High data rate (Mbps to Gbps)
- Long range (50-100 meters indoors)
- Internet connectivity

**Disadvantages:**
- High power consumption (>100 mW active)
- Larger antenna (λ/4 ≈ 31 mm at 2.4 GHz, 12 mm at 5 GHz)
- Authentication complexity

**Use Cases:**
- Home health monitoring (gateway to cloud)
- Rehabilitation devices with video streaming
- Smart home integration

#### 12.3.2 Cellular (LTE, 5G)

**Advantages:**
- Ubiquitous coverage
- High reliability
- Mobile connectivity

**Disadvantages:**
- Very high power consumption (>500 mW active)
- Subscription cost
- Larger module and antenna

**Applications:**
- Standalone medical devices (no smartphone required)
- Elderly care (fall detection → emergency call)
- Remote worker safety monitoring

### 12.4 Data Security and Privacy

#### 12.4.1 Encryption

**BLE Security:**
- Pairing: LE Secure Connections (ECDH key exchange)
- Encryption: AES-128-CCM
- Authentication: CMAC

**End-to-End Encryption:**
- TLS/SSL for internet communication
- Application-layer encryption for sensitive data

#### 12.4.2 Privacy Protection

**Data Minimization:**
- Collect only necessary data
- Anonymize or pseudonymize when possible

**User Consent:**
- Explicit opt-in for data collection
- Granular control (choose which data to share)

**Compliance:**
- GDPR (EU): Right to access, delete, portability
- HIPAA (US): Protected Health Information (PHI) safeguards
- Regional regulations: Ensure compliance in target markets

---

## 13. Manufacturing Processes

### 13.1 Fiber Production

#### 13.1.1 Coating Processes

**Electroless Plating:**
1. Surface activation (sensitization)
2. Immersion in plating bath (metal salt + reducing agent)
3. Metal deposition on fiber surface
4. Washing and drying

**Metals:** Silver, copper, nickel

**Physical Vapor Deposition (PVD):**
- Sputtering or evaporation in vacuum
- Uniform, thin coatings (10-1000 nm)
- High quality, expensive

**Dip Coating:**
- Immersion in conductive polymer or ink solution
- Withdrawal at controlled speed
- Drying/curing
- Thickness: 0.1-10 μm per coat

#### 13.1.2 Core-Sheath Fibers

**Structure:**
- Core: Conductive material (metal wire, CNT composite)
- Sheath: Protective polymer (polyester, nylon)

**Production:**
- Co-extrusion or wrapping
- Advantages: Protection, insulation, durability

#### 13.1.3 Blend Spinning

**Process:**
- Mix conductive and non-conductive fibers during spinning
- Conductive fiber fraction: 1-20%
- Homogeneous distribution

**Post-Treatment:**
- Carbonization (for carbon fibers)
- Chemical reduction (for graphene oxide)

### 13.2 Textile Fabrication

#### 13.2.1 Weaving

**Advantages:**
- High strength and stability
- Precise conductor placement in warp or weft
- Scalable for mass production

**Conductive Yarn Integration:**
- Warp: Longitudinal conductors
- Weft: Transverse conductors
- Jacquard weaving: Complex patterns

**Insulation:**
- Non-conductive yarns between conductive yarns
- Coating or lamination

#### 13.2.2 Knitting

**Advantages:**
- Excellent stretch and flexibility
- Seamless garment construction (3D knitting)
- Comfortable fit

**Conductive Integration:**
- Inlay knitting: Conductive yarn laid in without forming stitches
- Intarsia: Conductive regions knitted in
- Circular knitting: Tubular constructions for sleeves, legs

**Challenges:**
- Maintaining conductivity through complex stitch patterns
- Preventing short circuits in dense knits

#### 13.2.3 Embroidery

**Advantages:**
- Precise placement of conductive traces
- Multilayer capability (insulation between layers)
- Post-processing (add to existing garments)

**CNC Embroidery:**
- Conductive thread (silver-plated, stainless steel)
- Stitch density: 3-10 stitches/mm
- Line width: 0.5-2 mm

**Insulation:**
- Non-conductive thread layer
- Dielectric fabric overlay

**Applications:**
- Sensor electrodes
- Antennas
- Circuit traces

### 13.3 Printing Technologies

#### 13.3.1 Screen Printing

**Process:**
1. Prepare screen with pattern (photolithography or laser cut)
2. Apply conductive ink
3. Squeegee forces ink through screen onto fabric
4. Cure/dry (thermal, UV, or IR)

**Resolution:**
- Line width: 100-500 μm
- Layer thickness: 10-50 μm

**Conductive Inks:**
- Silver nanoparticle
- Carbon (graphene, CNT)
- PEDOT:PSS

**Advantages:**
- High throughput
- Low cost for mass production
- Thick layers (low resistance)

#### 13.3.2 Inkjet Printing

**Process:**
- Drop-on-demand inkjet head
- Digital pattern (no screen required)
- Cure/sinter after printing

**Resolution:**
- Line width: 50-200 μm
- Droplet size: 1-50 pL

**Advantages:**
- Rapid prototyping (no screen needed)
- Multi-material printing (sensors + circuits on same fabric)
- Low material waste

**Disadvantages:**
- Lower throughput than screen printing
- Thinner layers (higher resistance)

#### 13.3.3 Spray Coating

**Process:**
- Atomize ink into fine droplets
- Spray onto fabric (through mask or free-form)
- Dry/cure

**Applications:**
- Large area coating (conductive layers)
- Conformal coating on 3D shapes

**Advantages:**
- Simple, low equipment cost
- Good for prototyping

**Disadvantages:**
- Lower resolution
- Material waste (overspray)

### 13.4 Component Assembly

#### 13.4.1 Reflow Soldering

**Process:**
1. Apply solder paste to conductive pads
2. Place SMD components
3. Reflow oven (heat to melt solder)
4. Cool and solidify

**Temperature Profile:**
- Preheat: 150-180°C
- Soak: 180-200°C
- Reflow peak: 220-250°C (lead-free)
- Cooling: Gradual to prevent thermal shock

**Challenges:**
- Fabric thermal tolerance (<200°C for many textiles)
- Solution: Low-temperature solder (138-180°C), heat-resistant fabrics

#### 13.4.2 Conductive Adhesive

**Types:**
- Isotropic: Conductive in all directions
- Anisotropic (ACF/ACA): Conductive only in Z-axis

**Application:**
- Dispense adhesive on pad
- Place component
- Cure (thermal or UV)

**Advantages:**
- Lower temperature than soldering
- Flexible joints

**Disadvantages:**
- Lower conductivity than solder
- Longer cure time

#### 13.4.3 Snap Fasteners

**Design:**
- Male and female snap parts
- Conductive inner surface
- Removable components (for washing)

**Contact Resistance:**
- Target: <0.1 Ω
- Ensure good pressure and surface finish

**Applications:**
- Battery packs
- Sensor modules
- Control units

### 13.5 Quality Control

#### 13.5.1 Electrical Testing

**Continuity Test:**
- Four-wire resistance measurement
- Accept: R < threshold (e.g., <10 Ω for conductors)

**Insulation Test:**
- High voltage (100-500V) between isolated conductors
- Accept: R > 10 MΩ

**Functional Test:**
- Sensor output verification
- Signal integrity check

#### 13.5.2 Mechanical Inspection

**Visual Inspection:**
- Surface defects (holes, misalignment)
- Solder joint quality
- Component placement

**Dimensional Check:**
- Garment size and fit
- Conductor placement accuracy

#### 13.5.3 Wash Testing (Sampling)

**Pre-Production:**
- Prototype wash testing (10-50 cycles)
- Design validation

**Production:**
- Sample testing (e.g., 1 in 100 garments)
- Full wash cycle simulation

---

## 14. Testing and Certification

### 14.1 Electrical Safety Testing

#### 14.1.1 Voltage and Current Limits

**Safe Voltage Levels:**
- <12V DC: No shock hazard under normal conditions
- <50V DC: Safe for body contact in dry conditions
- >50V DC: Requires isolation and protection

**Current Limits:**
- <100 μA: No sensation
- 100 μA - 1 mA: Tingling sensation
- 1-10 mA: Painful, muscle contraction
- >10 mA: Dangerous, risk of fibrillation (AC)

**Design Guidelines:**
- Use low voltage (<12V) for body-contact applications
- Current limiting resistors
- Galvanic isolation for higher voltages

#### 14.1.2 Insulation Resistance

**Test:**
- Apply 500V DC between conductor and ground
- Measure insulation resistance

**Acceptance:**
- R_insulation > 10 MΩ (medical devices)
- R_insulation > 1 MΩ (consumer products)

#### 14.1.3 Leakage Current

**Test:**
- Connect device to mains (if applicable) or maximum voltage
- Measure leakage current to ground through body

**Limits (IEC 60601-1 for medical devices):**
- Normal condition: <100 μA
- Single fault condition: <500 μA

### 14.2 Biocompatibility Testing

#### 14.2.1 ISO 10993 Standards

**Test Categories:**
- Cytotoxicity: Cell viability in contact with material
- Sensitization: Allergic response
- Irritation: Skin and eye irritation
- Systemic toxicity: Acute and chronic effects
- Genotoxicity: DNA damage
- Implantation: For implantable devices (not typical for textiles)

**Device Contact Duration:**
- Limited: <24 hours
- Prolonged: 24 hours to 30 days
- Permanent: >30 days

**Skin Contact Classification:**
- Surface device, skin contact
- Typical textiles: Prolonged or permanent contact

#### 14.2.2 Skin Sensitization

**Patch Test:**
- Apply material to skin under occlusive patch
- 24-48 hour contact
- Evaluate for erythema, edema

**Common Allergens to Avoid:**
- Nickel (in stainless steel)
- Chromium (in leather tanning, some dyes)
- Latex (rubber components)

**Hypoallergenic Materials:**
- Medical-grade silicone
- Titanium, gold (conductive elements)
- Organic cotton (base fabric)

### 14.3 Electromagnetic Compatibility (EMC)

#### 14.3.1 Emissions

**Radiated Emissions:**
- Unintentional RF radiation
- Standards: FCC Part 15, EN 55011
- Limits: Prevent interference with other devices

**Conducted Emissions:**
- Noise on power lines
- Less relevant for battery-powered wearables

#### 14.3.2 Immunity

**Electrostatic Discharge (ESD):**
- IEC 61000-4-2
- Test levels: ±2 kV (contact), ±4 kV (air)
- Protection: ESD diodes, shielding

**Radiated Immunity:**
- IEC 61000-4-3
- Test with RF field (80-1000 MHz, 3-10 V/m)
- Ensure device functions correctly

### 14.4 Textile Performance Testing

#### 14.4.1 Colorfastness

**AATCC 61: Colorfastness to Laundering**
- Washing cycles with standard detergent
- Evaluate color change (1-5 scale, 5 = no change)
- Target: Grade 4-5

#### 14.4.2 Tensile Strength

**ASTM D5034: Breaking Strength**
- Grab test method
- Specimen: 100×150 mm
- Measure force at break (N)

**Acceptance:**
- Retention >90% after washing

#### 14.4.3 Air Permeability

**ASTM D737:**
- Measure airflow through fabric
- Units: cm³/s/cm² at 125 Pa
- Breathability: >20 cm³/s/cm² for comfort

#### 14.4.4 Moisture Management

**AATCC 195: Liquid Moisture Management**
- Wetting time (top and bottom)
- Absorption rate
- Spreading speed
- One-way transport capability

### 14.5 Clinical Validation

#### 14.5.1 Accuracy Assessment

**Comparison with Gold Standard:**
- ECG: Compare with clinical ECG (12-lead)
- Heart rate: Compare with pulse oximeter
- Temperature: Compare with clinical thermometer

**Metrics:**
- Sensitivity: TP / (TP + FN)
- Specificity: TN / (TN + FP)
- Bland-Altman plot: Agreement analysis
- Correlation coefficient: r > 0.90 for good agreement

#### 14.5.2 User Studies

**Comfort Assessment:**
- Questionnaires (Likert scale 1-5)
- Wear time and compliance
- Skin irritation reports

**Usability Testing:**
- Ease of donning/doffing
- Pairing and connectivity
- Data interpretation

**Sample Size:**
- Pilot study: 10-30 participants
- Clinical validation: 50-200 participants
- Multi-center trials: >500 participants

### 14.6 Regulatory Approval

#### 14.6.1 Medical Device Classification

**FDA (US):**
- Class I: Low risk (general controls)
- Class II: Moderate risk (510(k) clearance)
- Class III: High risk (PMA approval)

**Most smart textiles for health monitoring: Class II**

**EU MDR (Medical Device Regulation):**
- Class I: Low risk (self-certification)
- Class IIa/IIb: Medium risk (Notified Body)
- Class III: High risk (Notified Body + clinical data)

#### 14.6.2 Certification Bodies

**US:**
- FDA: Medical device approval
- FCC: Wireless communication compliance

**EU:**
- CE Mark: Conformité Européenne
- Notified Bodies for medical devices

**International:**
- ISO certifications (ISO 13485 for medical device QMS)

---

## 15. Safety and Compliance

### 15.1 Electrical Safety

#### 15.1.1 Design for Safety

**Low Voltage Design:**
- Operate at <12V DC whenever possible
- Use isolated power supplies for >50V

**Current Limiting:**
- Series resistors or current-limiting ICs
- Target: <100 μA through body

**Overcurrent Protection:**
- Fuses or PTC resettable fuses
- Prevent battery overheating

**Short Circuit Protection:**
- Automatic cutoff circuits
- Fail-safe design

#### 15.1.2 Battery Safety

**Lithium Battery Hazards:**
- Overcharge: Thermal runaway, fire
- Over-discharge: Capacity loss, swelling
- Puncture: Short circuit, fire
- High temperature: Degradation, fire

**Protection Circuits:**
- Battery Management System (BMS)
  - Overcharge protection (>4.2V per cell)
  - Over-discharge protection (<3.0V per cell)
  - Overcurrent protection (>2-3C)
  - Temperature monitoring (cutoff >60°C)

**Enclosure Design:**
- Rigid protective case
- Ventilation (prevent pressure buildup)
- Fire-resistant materials

#### 15.1.3 Standards Compliance

**IEC 60601-1:**
- Medical electrical equipment - General safety

**IEC 62133:**
- Secondary cells and batteries - Safety requirements

**UL 2054:**
- Household and commercial batteries

### 15.2 Mechanical Safety

#### 15.2.1 Sharp Edges and Protrusions

**Design Guidelines:**
- Round all edges (radius >1 mm)
- Encapsulate rigid components
- Smooth surfaces

**Testing:**
- Visual and tactile inspection
- No sharp edges that could cut skin or fabric

#### 15.2.2 Choking Hazards

**Detachable Components:**
- Secure attachment (withstand >10 N pull force)
- Warning labels for small parts

**For Children's Garments:**
- No removable parts <31 mm diameter (CPSC small parts regulation)

### 15.3 Thermal Safety

#### 15.3.1 Skin Contact Temperature

**Safe Temperature Limits:**
- Comfortable: 30-34°C (skin temperature)
- Warm but safe: 35-41°C
- Painful/burn risk: >43°C (prolonged contact)
- Immediate burn: >55°C

**Design Limits:**
- Continuous contact: <41°C
- Short contact (<10 min): <45°C
- Momentary contact: <55°C

**Heated Garments:**
- Temperature control with thermostats
- Maximum setpoint: 40-42°C
- Overtemperature cutoff: >45°C

#### 15.3.2 Thermal Runaway Prevention

**Battery Thermal Management:**
- Temperature sensors on battery pack
- Cutoff at >60°C
- Thermal fuse (backup)

**Heat Dissipation:**
- Adequate ventilation
- Heat sinks for high-power components

### 15.4 Chemical Safety

#### 15.4.1 Restricted Substances

**REACH (EU):**
- Registration, Evaluation, Authorization of Chemicals
- Restrict heavy metals (Pb, Cd, Hg, Cr VI)
- Phthalates, certain flame retardants

**RoHS (Restriction of Hazardous Substances):**
- Lead, mercury, cadmium, hexavalent chromium
- Polybrominated biphenyls (PBB), polybrominated diphenyl ethers (PBDE)

**California Prop 65:**
- Warning for carcinogens and reproductive toxins

#### 15.4.2 Material Toxicity

**Avoid:**
- Lead in solder (use lead-free)
- Cadmium in pigments
- Formaldehyde in resins (textile finishes)

**Safe Alternatives:**
- Tin-silver-copper (SAC) solder
- Organic pigments
- Formaldehyde-free resins

### 15.5 Data Privacy and Security

#### 15.5.1 Data Protection Regulations

**GDPR (General Data Protection Regulation) - EU:**
- Lawful basis for data processing
- User consent (explicit, informed)
- Right to access, rectify, delete data
- Data portability
- Breach notification (<72 hours)

**HIPAA (Health Insurance Portability and Accountability Act) - US:**
- Protected Health Information (PHI) safeguards
- Access controls
- Audit trails
- Encryption in transit and at rest

**CCPA (California Consumer Privacy Act) - US:**
- Right to know what data is collected
- Right to delete
- Right to opt-out of sale

#### 15.5.2 Security Best Practices

**Data Encryption:**
- AES-128 or AES-256 for stored data
- TLS 1.2+ for network transmission

**Authentication:**
- User authentication (password, biometric)
- Device authentication (unique ID, certificates)

**Access Control:**
- Role-based access control (RBAC)
- Principle of least privilege

**Secure Development:**
- Code review and security audits
- Penetration testing
- Regular software updates (patch vulnerabilities)

### 15.6 Labeling and User Instructions

#### 15.6.1 Care Labels

**Washing Instructions:**
- Symbols per ISO 3758
- Temperature, cycle type
- Special instructions (remove electronics, hand wash only)

**Storage:**
- Temperature and humidity conditions
- Avoid direct sunlight

#### 15.6.2 Safety Warnings

**Required Information:**
- Electrical safety (voltage, current)
- Battery warnings (fire, chemical hazard)
- Choking hazard (if applicable)
- Not for medical diagnosis (if not FDA cleared)

**Example Label:**
```
WARNING:
- Remove all electronics before washing
- Do not expose to water while powered on
- Charge only with provided charger
- Do not use if damaged
- Not for medical diagnosis or treatment
```

#### 15.6.3 User Manual

**Contents:**
- Product description and features
- Setup and pairing instructions
- Usage guidelines
- Maintenance and care
- Troubleshooting
- Safety precautions
- Warranty and support contact

---

## 16. Future Directions

### 16.1 Emerging Materials

**Graphene and 2D Materials:**
- Exceptional conductivity and strength
- Future: Mass production cost reduction

**Conductive MOFs (Metal-Organic Frameworks):**
- Porous, tunable properties
- Chemical sensing applications

**Liquid Metal Conductors:**
- Gallium-indium alloys (room temperature liquid)
- Extreme stretchability (>800%)
- Self-healing properties

### 16.2 Advanced Sensors

**Biochemical Sensors:**
- Sweat metabolites (glucose, lactate, urea)
- Hormones (cortisol for stress)
- Biomarkers for disease detection

**Multimodal Sensors:**
- Single sensor detecting multiple parameters
- Reduced complexity and cost

**Implantable-Grade Accuracy:**
- Wearable sensors approaching clinical accuracy
- Regulatory approval for medical use

### 16.3 AI and Machine Learning

**On-Device AI:**
- Edge computing on wearable MCU
- Real-time pattern recognition (arrhythmia, fall)
- Privacy (data stays on device)

**Personalized Models:**
- Adapt to individual user physiology
- Improved accuracy over time

**Predictive Analytics:**
- Predict health events (seizures, cardiac events)
- Early warning systems

### 16.4 Sustainability

**Biodegradable Materials:**
- Cellulose-based conductive fibers
- Natural polymers

**Recycling:**
- Design for disassembly
- Material recovery (precious metals)

**Low-Impact Manufacturing:**
- Water-free dyeing
- Renewable energy in production

### 16.5 Standardization Efforts

**International Collaboration:**
- ISO/IEC standards for smart textiles
- Interoperability (devices from different manufacturers)

**Open Protocols:**
- Data formats and APIs
- Enable ecosystem development

---

## Appendix A: Reference Formulas

### Electrical Properties

**Conductivity:**
```
σ = 1 / ρ
σ (S/m), ρ (Ω·m)
```

**Resistance of Conductor:**
```
R = ρ × L / A
L = length (m), A = cross-sectional area (m²)
```

**Gauge Factor:**
```
GF = (ΔR/R₀) / ε
```

### Thermal Properties

**Heat Transfer:**
```
Q = h × A × ΔT
h = heat transfer coefficient (W/m²·K)
A = area (m²)
ΔT = temperature difference (K)
```

**PCM Energy Storage:**
```
Q = m × (c_p × ΔT + L_f)
```

### Power and Energy

**Electrical Power:**
```
P = V × I = I² × R = V² / R
```

**Energy:**
```
E = P × t
```

**Battery Capacity:**
```
Capacity (Wh) = Voltage (V) × Capacity (Ah)
```

---

## Appendix B: Glossary

See Section 4 for detailed terms and definitions.

---

## Appendix C: References

### Academic Literature
1. Stoppa, M., & Chiolerio, A. (2014). Wearable electronics and smart textiles: a critical review. *Sensors*, 14(7), 11957-11992.
2. 선행 연구. Fiber-based wearable electronics: A review of materials, fabrication, devices, and applications. *Advanced Materials*, 26(31), 5310-5336.
3. Cherenack, K., & Van Pieterson, L. (2012). Smart textiles: Challenges and opportunities. *Journal of Applied Physics*, 112(9), 091301.

### Standards Organizations
- ISO (International Organization for Standardization)
- IEC (International Electrotechnical Commission)
- ASTM International
- AATCC (American Association of Textile Chemists and Colorists)

### Regulatory Bodies
- FDA (Food and Drug Administration) - US
- EMA (European Medicines Agency) - EU
- FCC (Federal Communications Commission) - US

---

## Document Revision History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2025-01-15 | Initial release | WIA Industrial Research Group |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
