# PHASE 3 — Protocol

> Wearable-fashion operational protocols: power systems and
> energy budgeting, smart materials behavior, thermal management,
> and the manufacturing-and-assembly protocol that produces
> conformant garments at scale.

## 8. Power Systems

### 8.1 Battery Technologies

#### 8.1.1 Lithium Polymer (LiPo)

```
Specifications:
- Voltage: 3.7V nominal (3.0-4.2V range)
- Capacity: 100-5000mAh (wearables)
- Energy density: 150-250 Wh/kg
- Discharge rate: 1C-20C
- Cycle life: 300-500 cycles
- Form factor: Flexible pouch
- Safety: Requires protection circuit
```

**Advantages:**
- Lightweight
- Flexible form factor
- High capacity options

**Disadvantages:**
- Requires careful handling
- Fire risk if damaged
- Limited cycle life

#### 8.1.2 Flexible Battery

```
Specifications:
- Voltage: 3.7V nominal
- Capacity: 50-500mAh
- Thickness: 0.4-2mm
- Flexibility: Can bend to 20mm radius
- Cycle life: 300-500 cycles
- Temperature range: -10°C to +50°C
```

**Applications:**
- Garments requiring flexibility
- Washable designs (when properly sealed)
- Comfortable wearables

#### 8.1.3 Coin Cell

```
Common Types:
- CR2032: 3V, 220mAh, 20mm diameter
- CR2025: 3V, 165mAh, 20mm diameter
- CR1632: 3V, 125mAh, 16mm diameter

Characteristics:
- Non-rechargeable (primary)
- Long shelf life (10 years)
- Stable voltage
- Low self-discharge
```

**Applications:**
- Low-power smart jewelry
- Emergency backup power
- Simple LED accessories

### 8.2 Charging Methods

#### 8.2.1 USB Charging

**USB-C (Recommended)**
```
Specifications:
- Voltage: 5V (USB 2.0 Power Delivery)
- Current: 500mA-3A
- Connector: Reversible, robust
- Data: Optional
- Waterproofing: Requires port cover or gasket
```

**Micro USB (Legacy)**
```
Specifications:
- Voltage: 5V
- Current: 500mA-2A
- Durability: ~5000 insertions
- Waterproofing: Difficult
```

#### 8.2.2 Wireless Charging

**Qi Standard**
```
Specifications:
- Power: 5-15W
- Efficiency: 70-80%
- Distance: 5-40mm
- Frequency: 110-205 kHz
- Alignment: Critical (±5mm)
```

**Implementation:**
- Receiver coil integrated into garment
- Transmitter pad for charging
- Foreign object detection
- Automatic power adjustment

#### 8.2.3 Magnetic Connector

```
Specifications:
- Pins: 2-4 (power, ground, data)
- Current: Up to 3A
- Magnetic force: 50-200g
- Waterproofing: Excellent (IP67-IP68)
- Alignment: Self-aligning
```

**Advantages:**
- Easy connection/disconnection
- Excellent waterproofing
- No port wear

### 8.3 Energy Harvesting

#### 8.3.1 Solar Energy

**Flexible Solar Cells**
```
Specifications:
- Efficiency: 15-23%
- Power density: 100-250 mW/10cm²
- Voltage: 0.5-6V (depending on cells in series)
- Flexibility: Bend radius >20mm
- Durability: >10,000 flex cycles
```

**Calculation:**
```
Power (mW) = Area (cm²) × Irradiance (mW/cm²) × Efficiency

Example:
200 cm² × 100 mW/cm² × 0.20 = 4000mW = 4W (peak)

Realistic outdoor average: 1-2W
Indoor lighting: 0.1-0.5W
```

#### 8.3.2 Kinetic Energy

**Piezoelectric Harvesting**
```
Power output: 0.1-10mW per element
Response time: <1ms
Voltage: 1-20V (requires regulation)
Placement: High-movement areas (shoulders, elbows, knees)
```

**Electromagnetic Induction**
```
Power output: 1-100mW
Requires: Magnetic field + motion
Efficiency: 20-40%
Applications: Walking, arm movement
```

#### 8.3.3 Thermoelectric

**Body Heat Harvesting**
```
Temperature differential: 5-10°C (skin to ambient)
Power output: 1-50mW per cm²
Efficiency: 3-8%
TEG voltage: 0.1-1V
```

**Applications:**
- Wristbands
- Headbands
- Areas with good skin contact

### 8.4 Power Distribution

#### 8.4.1 Conductive Pathways

**Conductive Thread**
```
Properties:
- Resistance: 0.1-30 Ω/m
- Current capacity: 0.5-1A
- Flexibility: Excellent
- Washability: Good (with proper technique)
- Materials: Silver, copper, stainless steel
```

**Conductive Fabric Traces**
```
Properties:
- Resistance: 0.01-1 Ω/sq
- Current capacity: 1-5A (depending on width)
- Width: 2-20mm typical
- Application: Screen printing, weaving
```

**Flexible PCB**
```
Properties:
- Thickness: 0.1-0.3mm
- Layers: 1-4
- Bend radius: 5-20mm
- Durability: >100,000 flex cycles
```

#### 8.4.2 Power Regulation

**Buck Converter (Step-down)**
```
Input: 5-12V
Output: 3.3V or 5V
Efficiency: 85-95%
Ripple: <50mV
Load regulation: ±2%
```

**Linear Regulator**
```
Input: 4-20V
Output: 3.3V or 5V
Efficiency: 50-85% (voltage dependent)
Noise: Very low
Applications: Sensitive sensors
```

### 8.5 Battery Life Calculations

#### 8.5.1 Simple Calculation

```
Battery Life (hours) = Battery Capacity (mAh) / Average Current (mA)

Example:
500mAh battery, 50mA average current
Battery life = 500 / 50 = 10 hours
```

#### 8.5.2 Detailed Calculation

```
Total Current = LED Current + Sensor Current + MCU Current + Wireless Current

LED Current = (Number of LEDs × Current per LED × Average Brightness × Duty Cycle)
Sensor Current = (Sensor Current × Sample Rate / Max Rate)
MCU Current = (Active Current × Active % + Sleep Current × Sleep %)
Wireless Current = (TX Current × TX % + RX Current × RX % + Idle Current × Idle %)

Battery Life = (Battery Capacity × DoD × Efficiency) / Total Current
```

**Example:**
```
Battery: 1000mAh LiPo
LEDs: 50 × 20mA × 0.5 brightness × 0.3 duty cycle = 150mA
Sensors: 5mA × (1Hz / 100Hz) = 0.05mA
MCU: 8mA × 0.3 + 0.05mA × 0.7 = 2.44mA
Bluetooth: 15mA × 0.01 + 5mA × 0.09 + 0.5mA × 0.9 = 1.05mA

Total: 150 + 0.05 + 2.44 + 1.05 = 153.54mA

Battery Life = (1000 × 0.9 × 0.95) / 153.54 = 5.57 hours
```

---


## 9. Smart Materials

### 9.1 Conductive Textiles

#### 9.1.1 Conductive Thread Types

**Silver-Plated Thread**
```
Specifications:
- Resistance: 0.1-1 Ω/m
- Core: Nylon or polyester
- Coating: 99% silver
- Diameter: 0.2-0.5mm
- Current capacity: 0.5-1A
- Washing: Hand wash recommended
- Oxidation: Moderate (can tarnish)
```

**Stainless Steel Thread**
```
Specifications:
- Resistance: 10-30 Ω/m
- Material: 316L stainless steel
- Diameter: 0.1-0.3mm
- Current capacity: 0.3-0.5A
- Washing: Machine washable
- Durability: Excellent
- Cost: Low
```

**Copper Thread**
```
Specifications:
- Resistance: 0.5-5 Ω/m
- Core: Polymer
- Coating: Copper
- Diameter: 0.2-0.4mm
- Current capacity: 0.5-0.8A
- Oxidation: High (requires coating)
```

#### 9.1.2 Conductive Fabrics

**Silver-Coated Fabric**
```
Properties:
- Sheet resistance: 0.01-1 Ω/sq
- Shielding effectiveness: >60dB (EMI)
- Stretchability: Minimal
- Applications: Touch sensors, EMI shielding
```

**Conductive Polymer Fabric**
```
Properties:
- Sheet resistance: 1-100 Ω/sq
- Stretch: Up to 200%
- Washability: Excellent
- Applications: Stretch sensors, flexible circuits
```

### 9.2 Thermal Materials

#### 9.2.1 Heating Materials

**Carbon Fiber Heating Elements**
```
Specifications:
- Power density: 0.5-2 W/cm²
- Resistance: 20-100 Ω/m
- Maximum temperature: 45-60°C
- Response time: 1-3 minutes
- Flexibility: Good
- Efficiency: >95%
```

**Resistance Wire (Nichrome)**
```
Specifications:
- Resistivity: 1.1 Ω·mm²/m
- Maximum temp: 50-100°C (garment applications)
- Power: 1-5W per element
- Form: Wire (28-36 AWG) or ribbon
```

#### 9.2.2 Cooling Materials

**Phase Change Materials (PCM)**
```
Properties:
- Phase transition: 18-28°C (comfort range)
- Latent heat: 150-250 J/g
- Encapsulation: Microencapsulated
- Duration: 1-4 hours
- Rechargeable: Yes (by temperature change)
```

**Moisture-Wicking Fabrics**
```
Properties:
- Evaporative cooling
- Moisture transfer rate: >3000 g/m²/24h
- Drying time: <2 hours
- Materials: Polyester, nylon, merino wool blends
```

### 9.3 Sensing Materials

#### 9.3.1 Pressure Sensors

**Piezoresistive Fabric**
```
Properties:
- Resistance change: 10-1000% under pressure
- Sensing area: 1-100cm²
- Response time: <10ms
- Pressure range: 1kPa-1MPa
- Applications: Touch interfaces, posture sensing
```

**Capacitive Sensing**
```
Properties:
- Capacitance change: 1-100pF
- Sensitivity: Can detect through 5mm fabric
- Noise immunity: Good
- Applications: Touch buttons, proximity
```

#### 9.3.2 Stretch Sensors

**Conductive Elastomer**
```
Properties:
- Strain range: 0-300%
- Resistance change: Linear or exponential
- Response time: <50ms
- Hysteresis: <10%
- Applications: Motion capture, fit detection
```

---


## 10. Thermal Management

### 10.1 Heating Systems

#### 10.1.1 Heating Element Design

**Zone Heating**
```
Body Zones:
- Core (chest, back): 10-20W, 35-40°C
- Extremities (hands, feet): 5-10W, 30-35°C
- Neck: 3-5W, 30-35°C

Total power budget: 20-35W typical
Battery: 5000-10000mAh for 2-4 hour operation
```

**Temperature Control**
```
Control methods:
1. PWM (Pulse Width Modulation)
   - Frequency: 1-100Hz
   - Resolution: 8-bit (256 levels)
   - Efficiency: >95%

2. Bang-bang (on/off)
   - Simple implementation
   - Temperature ripple: ±2°C
   - Efficiency: >98%

3. PID control
   - Precise temperature
   - Complex implementation
   - Adaptive to conditions
```

#### 10.1.2 Safety Features

**Thermal Cutoff**
```
Maximum temperatures:
- Skin contact: 45°C continuous, 50°C peak
- Internal: 60°C
- Battery proximity: 40°C

Implementation:
- NTC thermistor: 10kΩ at 25°C
- Thermal fuse: 60-70°C trip
- Microcontroller monitoring: 1Hz sample rate
- Automatic shutdown: >45°C for >5 minutes
```

### 10.2 Cooling Systems

#### 10.2.1 Active Cooling

**Peltier Modules**
```
Specifications:
- Cooling power: 2-10W
- Voltage: 5-12V
- Current: 0.5-3A
- Temperature differential: 5-20°C
- Efficiency (COP): 0.3-0.8
- Size: 20×20mm to 40×40mm
```

**Micro Fans**
```
Specifications:
- Size: 15-30mm diameter
- Airflow: 0.5-5 CFM
- Noise: 20-35 dBA
- Power: 0.5-2W
- Voltage: 3.3-5V
- Speed control: PWM
```

#### 10.2.2 Passive Cooling

**Heat Dissipation Fabrics**
```
Properties:
- Thermal conductivity: 0.1-0.5 W/m·K
- Air permeability: >100 mm/s
- Moisture management: Wicking
- Materials: Polyester blends, mesh structures
```

### 10.3 Thermal Comfort Calculations

#### 10.3.1 Heat Transfer

```
Heat Transfer Rate (W) = U × A × ΔT

Where:
U = Overall heat transfer coefficient (5-25 W/m²·K for clothing)
A = Surface area (m²)
ΔT = Temperature difference (°C)

Example:
U = 15 W/m²·K
A = 0.5 m² (chest heating pad)
ΔT = 10°C (35°C target - 25°C ambient)

Heat = 15 × 0.5 × 10 = 75W (required)

With insulation:
Heat = 75W × 0.3 = 22.5W (actual needed)
```

#### 10.3.2 Heating Time

```
Time (seconds) = (m × c × ΔT) / P

Where:
m = Mass (kg)
c = Specific heat capacity (J/kg·K)
ΔT = Temperature change (°C)
P = Heating power (W)

Example:
m = 0.5 kg (fabric + heating element)
c = 1500 J/kg·K (textile average)
ΔT = 15°C (20°C to 35°C)
P = 20W

Time = (0.5 × 1500 × 15) / 20 = 562.5 seconds ≈ 9.4 minutes
```

---


## 13. Manufacturing and Assembly

### 13.1 Production Methods

#### 13.1.1 Component Attachment

**Sewing**
```
Method: Hand or machine sewing
Materials: Conductive thread for connections
Advantages:
  - Traditional technique
  - No special equipment
  - Repairable
Disadvantages:
  - Time-consuming
  - Skill-dependent
  - Limited precision

Stitch types:
  - Running stitch: Simple connections
  - Backstitch: Stronger mechanical bond
  - Zigzag: Flexible connections
```

**Adhesive Bonding**
```
Adhesives:
  - Textile adhesive: Washable, flexible
  - Epoxy: Strong, rigid
  - Silicone: Waterproof, flexible
  - Conductive adhesive: For electrical connections

Process:
  1. Surface preparation
  2. Adhesive application (screen printing, manual)
  3. Component placement
  4. Curing (heat, UV, or time)

Advantages: Fast, precise, automated
Disadvantages: Less repairable, curing time
```

**Ultrasonic Welding**
```
Process: High-frequency vibration creates heat, bonds materials
Frequency: 20-40 kHz
Applications: Synthetic fabrics (polyester, nylon)
Advantages:
  - Fast (< 1 second)
  - No consumables
  - Strong bond
  - Sealed seams (waterproof)
```

#### 13.1.2 Circuit Integration

**Embroidered Circuits**
```
Method: Computerized embroidery with conductive thread
Precision: ±0.5mm
Trace width: 1-5mm
Resistance: Depends on thread (0.1-30 Ω/m)
Applications:
  - Simple circuits
  - Antennas
  - Touch sensors
```

**Printed Electronics**
```
Methods:
  - Screen printing
  - Inkjet printing
  - 3D printing

Inks:
  - Silver (high conductivity, expensive)
  - Copper (good conductivity, oxidation)
  - Carbon (moderate conductivity, cheap)

Applications:
  - Flexible PCBs
  - Sensors
  - Heating elements
```

**Woven Circuits**
```
Method: Conductive fibers woven into fabric structure
Integration: Warp or weft threads
Complexity: Limited to simple circuits
Advantages:
  - Inherently flexible
  - Durable
  - Washable
```

### 13.2 Assembly Process

#### 13.2.1 Standard Assembly Flow

```
1. Fabric Cutting
   ↓
2. Circuit Integration (embroidery/printing/weaving)
   ↓
3. Component Attachment (LEDs, sensors, etc.)
   ↓
4. Power System Integration
   ↓
5. Functional Testing
   ↓
6. Garment Assembly (sewing pieces together)
   ↓
7. Final Testing & QC
   ↓
8. Waterproofing/Encapsulation (if applicable)
   ↓
9. Packaging
```

#### 13.2.2 Quality Control Points

**Visual Inspection**
- Component orientation
- Solder quality (if applicable)
- Thread tension and routing
- Fabric defects

**Electrical Testing**
- Continuity testing
- Resistance measurement
- Insulation testing
- Functional testing (LEDs, sensors)

**Mechanical Testing**
- Flex testing
- Pull strength
- Abrasion resistance
- Seam strength

**Environmental Testing**
- Water resistance (IP rating)
- Temperature cycling
- Humidity exposure

### 13.3 Waterproofing

#### 13.3.1 IP Ratings

```
IP Code: IP XY
  X = Dust protection (0-6)
  Y = Water protection (0-9K)

Common ratings for wearables:
- IP54: Dust protected, splash resistant
- IP65: Dust tight, water jets
- IP67: Dust tight, temporary immersion (1m, 30min)
- IP68: Dust tight, continuous immersion (manufacturer specified)
```

#### 13.3.2 Waterproofing Methods

**Conformal Coating**
```
Materials: Acrylic, silicone, polyurethane, parylene
Thickness: 25-250 μm
Application: Spray, dip, brush
Protection: Moisture, dust, chemicals
Allows washing: Yes (hand wash typically)
```

**Encapsulation**
```
Materials: Silicone, polyurethane, epoxy
Method: Potting, overmolding
Protection: Complete sealing
Repairability: Difficult
Applications: Critical electronics (battery, controller)
```

**Sealed Connectors**
```
Types: O-ring sealed, overmolded, magnetic
Rating: Up to IP68
Applications: Charging ports, external connections
```

---


