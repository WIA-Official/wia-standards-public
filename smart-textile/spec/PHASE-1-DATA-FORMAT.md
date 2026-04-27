# PHASE 1 — Data Format

> Smart-textile canonical envelopes: e-textile architecture
> shapes, terminology, scope, normative references, and the
> reference formulas, glossary, and references that define how
> textile data is interchanged across manufacturing, testing,
> wear-time, and disposal phases.

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

