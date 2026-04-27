# PHASE 3 — Protocol

> Smart-textile autonomic protocols: temperature regulation,
> energy harvesting, and the wireless communication layer that
> ties body-mounted textiles to gateway hubs without imposing
> unbearable RF burden on the wearer.

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


