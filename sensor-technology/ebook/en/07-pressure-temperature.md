# Chapter 7: Advanced Pressure and Temperature Sensing

## High-Performance Pressure Sensing

### Automotive Pressure Sensors

#### Engine Management Pressure Sensors

**Manifold Absolute Pressure (MAP) Sensor:**
```
Purpose: Measure intake manifold pressure
Range: 20-250 kPa (absolute)
Accuracy: ±2% full scale
Response time: <10 ms
Temperature range: -40°C to +125°C
```

**Operating Principle:**
- MEMS piezoresistive silicon diaphragm
- Wheatstone bridge configuration
- Integrated signal conditioning ASIC
- Temperature compensation via on-chip sensor and calibration coefficients

**TE Connectivity MS4525DO:**
- Differential pressure sensor
- Range: ±1 psi to ±30 psi options
- I2C output (14-bit)
- Accuracy: ±0.25% FSS
- Automotive-grade (-40°C to +85°C)

**Fuel Rail Pressure Sensor:**
```
Purpose: Direct injection fuel pressure monitoring
Range: 0-20 MPa (0-200 bar, 0-2900 psi)
Accuracy: ±1%
Media compatibility: Gasoline, diesel, ethanol blends
Safety: ASIL-C rated
```

**Design Challenges:**
- Harsh media (corrosive fuel)
- High pressure (steel diaphragm, thick-film resistors)
- Vibration resistance
- Long-term stability (10+ years)

**Bosch HDP5 High-Pressure Sensor:**
- Pressure range: Up to 250 MPa (advanced diesel)
- Temperature compensation: -40°C to +150°C
- Analog output: 0.5-4.5V ratiometric
- Media: Fuel, brake fluid, hydraulic oil

#### Tire Pressure Monitoring System (TPMS)

**Direct TPMS:**
- Sensor inside each tire
- Measures pressure + temperature
- Wireless transmission (315/433 MHz)
- Battery-powered (5-10 year lifetime)

**Infineon SP37:**
- Pressure range: 100-450 kPa
- Accuracy: ±1.5 kPa
- Temperature: -40°C to +125°C
- Integrated RF transmitter
- Low power: <15 μA average (1 min TX interval)
- Acceleration sensor for rotation detection

**Regulations:**
- USA: FMVSS 138 (mandatory since 2008)
- EU: ECE R64 (mandatory since 2014)
- Warning threshold: 25% below recommended pressure

**Indirect TPMS:**
- No pressure sensors
- Uses ABS wheel speed sensors
- Underinflated tire has smaller diameter → rotates faster
- Lower cost, but less accurate

**Next Generation: Bluetooth TPMS:**
- Bluetooth Low Energy (BLE)
- Smartphone app integration
- Continuous monitoring, detailed analytics
- Aftermarket retrofit kits

### Medical Pressure Sensors

#### Blood Pressure Monitoring

**Sphygmomanometer (Oscillometric Method):**
1. Inflatable cuff occludes brachial artery
2. Pressure sensor in cuff measures oscillations
3. Microcontroller analyzes waveform
4. Detect systolic, diastolic, mean arterial pressure

**TE Connectivity MS5837-30BA:**
- Pressure range: 0-30 bar (underwater, hyperbaric)
- Resolution: 0.2 mbar (2 mm H₂O)
- I2C/SPI interface
- Gel-filled port for liquid media
- Biocompatible materials

**Continuous Blood Pressure Monitoring:**
- Arterial line (invasive): Direct measurement via catheter
- Pressure transducer: 0-300 mmHg range
- High fidelity: Capture pulse waveform (120 Hz sampling)
- ICU, operating room use

**Non-Invasive Continuous:**
- Photoplethysmography (PPG) + algorithms
- Smartwatch/wearable implementations
- Accuracy: ±5-15 mmHg (not medical-grade yet)
- Research: Pulse transit time, machine learning models

#### Respiratory Pressure Sensors

**Mechanical Ventilators:**
```
Parameters measured:
- Airway pressure: 0-100 cmH₂O
- Flow rate: 0-200 L/min (via differential pressure across orifice)
- Tidal volume: Integration of flow
- PEEP (Positive End-Expiratory Pressure)
```

**Sensirion SDP800:**
- Differential pressure sensor
- Range: ±125 Pa to ±500 Pa
- Accuracy: ±3% reading
- Digital I2C interface
- Medical-grade certification (ISO 13485)
- Autoclavable package options

**CPAP (Continuous Positive Airway Pressure):**
- Sleep apnea treatment
- Maintain 4-20 cmH₂O pressure
- Pressure control loop (PID)
- Leak detection and compensation

#### Bariatric and Wearable Sensors

**Smart Insoles:**
- Pressure mapping for gait analysis
- Multiple sensors per insole (6-16 points)
- Diabetic foot ulcer prevention
- Sports performance optimization

**FSR (Force-Sensitive Resistor):**
- Polymer thick film on flexible substrate
- Resistance decreases with applied force
- Low cost (<$5)
- Limited accuracy, hysteresis
- Applications: Presence detection, relative pressure

**Capacitive Pressure Sensors:**
- Compressible dielectric between electrodes
- Linear response, better accuracy than FSR
- Higher cost, complex fabrication

### Industrial Pressure Applications

#### Process Control

**Pressure Transmitters:**
```
Standard signals:
- 4-20 mA current loop (analog)
- HART (Highway Addressable Remote Transducer) - digital over 4-20 mA
- Foundation Fieldbus, PROFIBUS (digital protocols)

Accuracy classes:
- Class 0.25: ±0.25% of span
- Class 0.1: High-precision applications
```

**Rosemount 3051:**
- Industry-standard differential pressure transmitter
- Range: 0-10 kPa to 0-40 MPa
- Accuracy: ±0.04% of span
- Temperature: -40°C to +85°C
- Explosive atmosphere certifications (ATEX, IECEx)

**Pressure Calibration:**
- Deadweight tester (primary standard)
- Pressure controller + reference gauge
- Automated calibration systems
- NIST-traceable standards

#### Level Measurement

**Hydrostatic Level:**
```
Pressure = ρ × g × h

Where:
ρ = Fluid density (kg/m³)
g = Gravitational acceleration (9.81 m/s²)
h = Liquid height (m)

Example:
Water tank, h = 5 m
P = 1000 × 9.81 × 5 = 49,050 Pa ≈ 0.49 bar
```

**Submersible Pressure Transmitter:**
- Stainless steel housing, IP68
- Vented cable (atmospheric reference)
- Range: 0-10 m to 0-500 m H₂O
- Applications: Wells, tanks, wastewater

**Challenges:**
- Density variation with temperature
- Foam on liquid surface
- Viscous or corrosive media
- Calibration with actual fluid

### Temperature Sensing Advanced Topics

#### Ultra-Precision Temperature Measurement

**Standard Platinum Resistance Thermometer (SPRT):**
- Traceable to ITS-90 temperature scale
- Uncertainty: ±0.001°C (0°C), ±0.005°C (100°C)
- Four-wire measurement (eliminates lead resistance)
- Slow response (thermal mass)
- Cost: $500-5000

**Thermistor Interchangeability:**
```
Steinhart-Hart Equation:
1/T = A + B×ln(R) + C×(ln(R))³

Where:
T = Temperature (Kelvin)
R = Resistance (Ω)
A, B, C = Steinhart-Hart coefficients

Accuracy:
- ±0.01°C over 0-70°C (precision thermistors)
- Requires individual calibration
```

**Interchangeable Thermistors:**
- Pre-calibrated, no individual characterization
- Tolerance: ±0.1°C to ±0.2°C
- Cost premium: 2-3x vs. standard
- Example: Murata NCP15XH103

#### Infrared (Non-Contact) Temperature Sensors

**Thermopile IR Sensors:**

**Stefan-Boltzmann Law:**
```
E = ε × σ × T⁴

Where:
E = Radiated power (W/m²)
ε = Emissivity (0-1)
σ = Stefan-Boltzmann constant (5.67 × 10⁻⁸ W/m²/K⁴)
T = Absolute temperature (K)
```

**Melexis MLX90614:**
- Non-contact IR thermometer IC
- Temperature range: -40°C to +125°C (object), up to 380°C versions
- Accuracy: ±0.5°C @ 0-50°C
- Field of view: 90° (standard), 10° (narrow versions)
- SMBus (I2C-like) interface
- Applications: Forehead thermometers, HVAC, automotive

**Emissivity Compensation:**
- Emissivity varies by material, surface finish, angle
  - Blackbody: ε = 1.0 (ideal)
  - Human skin: ε = 0.98
  - Polished aluminum: ε = 0.05 (highly reflective)
- Measurement error if emissivity unknown
- Solution: Known emissivity setting, or dual-wavelength pyrometers

**Thermographic Cameras:**
- Array of microbolometers (80×60 to 640×512 pixels)
- Uncooled (vanadium oxide, amorphous silicon)
- Resolution: 0.05°C temperature difference
- Frame rate: 9-60 Hz
- Cost: $200 (FLIR Lepton module) to $50,000+ (professional)

**Applications:**
- Building energy audits (heat loss detection)
- Electrical inspection (hot spots in panels)
- Predictive maintenance (bearing temperature)
- Medical: Fever screening, inflammation detection
- Firefighting: See through smoke

#### Fiber Optic Temperature Sensors

**Fiber Bragg Grating (FBG):**

**Principle:**
- Periodic variation in refractive index along fiber core
- Reflects specific wavelength (Bragg wavelength)
- Wavelength shifts with temperature and strain

**Temperature Sensitivity:**
```
Δλ/λ = α × ΔT + β × Δε

Where:
α = Thermal coefficient (~10 pm/°C)
β = Strain coefficient (~1.2 pm/με)
Δλ = Wavelength shift
ΔT = Temperature change
Δε = Strain
```

**Advantages:**
- Immune to electromagnetic interference
- Intrinsically safe (no electrical signal)
- Multiplexing: 10-100 sensors on single fiber
- Harsh environments: High temperature, radiation

**Applications:**
- Power transformer monitoring
- Turbine blade temperature
- Composite structure health monitoring (aircraft, wind turbines)
- Oil well downhole monitoring (up to 300°C)

**Distributed Temperature Sensing (DTS):**
- Raman scattering in optical fiber
- Entire fiber acts as sensor
- Spatial resolution: 1 m
- Distance: Up to 30 km
- Applications: Pipeline leak detection, fire detection in tunnels

### Emerging Pressure and Temperature Technologies

#### MEMS Resonant Pressure Sensors

**Principle:**
- Pressure changes resonant frequency of MEMS structure
- Frequency output (inherently digital)
- High resolution, low drift

**Frequency-Pressure Relationship:**
```
f = f₀ × √(1 + α × ΔP)

Where:
f = Resonant frequency
f₀ = Reference frequency (e.g., 100 kHz)
α = Pressure sensitivity coefficient
ΔP = Pressure change
```

**Advantages:**
- Noise-immune (frequency-based)
- Long-term stability
- No ADC required (frequency counter)

**Challenges:**
- More complex drive electronics
- Higher power (sustain resonance)
- Vacuum packaging for high Q-factor

**Bosch SMI (Smart Sensor Integration):**
- Research program
- Resonant MEMS pressure sensors
- Target: 10x better long-term stability

#### Flexible and Wearable Pressure Sensors

**Piezoresistive Fabric:**
- Conductive threads in textile
- Resistance changes with pressure/stretch
- Applications: Smart clothing, compression garments

**Capacitive E-skin:**
- Stretchable electrodes + dielectric
- Conformal to curved surfaces
- Robotic tactile sensing
- Prosthetic limb feedback

**Challenges:**
- Durability (washing, abrasion)
- Calibration (non-linear, hysteresis)
- Integration with electronics

#### Quantum Temperature Sensors

**Nitrogen-Vacancy (NV) Centers in Diamond:**
- Atomic-scale defects in diamond crystal
- Spin states sensitive to temperature, magnetic field
- Optically detected magnetic resonance (ODMR)

**Performance:**
- Sensitivity: <1 mK/√Hz
- Spatial resolution: <10 nm (nanoscale thermometry)
- Applications: Intracellular temperature mapping, nanoscale heat transfer

**Status:**
- Laboratory demonstrations
- Expensive (diamond synthesis, laser optics)
- 10+ years from commercialization

### Calibration Standards and Traceability

#### Temperature Calibration

**ITS-90 Fixed Points:**
| Point | Temperature (°C) | Uncertainty (mK) |
|-------|------------------|------------------|
| Triple point of hydrogen | -259.3467 | 0.5 |
| Boiling point of helium | -268.928 | 1.0 |
| Triple point of neon | -248.5939 | 2.0 |
| Triple point of oxygen | -218.7916 | 2.0 |
| Triple point of argon | -189.3442 | 2.0 |
| Triple point of mercury | -38.8344 | 1.0 |
| Triple point of water | 0.01 (exactly) | 0.1 |
| Melting point of gallium | 29.7646 | 1.0 |
| Freezing point of indium | 156.5985 | 2.0 |
| Freezing point of tin | 231.928 | 2.0 |
| Freezing point of zinc | 419.527 | 3.0 |
| Freezing point of aluminum | 660.323 | 5.0 |
| Freezing point of silver | 961.78 | 10 |
| Freezing point of gold | 1064.18 | 20 |
| Freezing point of copper | 1084.62 | 30 |

**Practical Calibration:**
- Ice bath: 0°C (±0.1°C with distilled water, crushed ice)
- Boiling water: 100°C at 1 atm (altitude correction)
- Commercial calibrators: Dry-block, liquid bath (±0.01°C to ±0.5°C)

#### Pressure Calibration

**Primary Standards:**
- Deadweight tester: Precision masses on piston
- Mercury manometer: Column height (mmHg)
- Controlled clearance piston gauge (highest accuracy)

**Secondary Standards:**
- Digital pressure gauge (Rosemount, Fluke)
- Uncertainty: 0.01% of reading
- NIST-traceable calibration certificate

**Portable Calibrators:**
- Hand pump + reference gauge
- Pneumatic (0-20 bar) or hydraulic (20-700 bar)
- Field calibration of installed sensors

### Integration and System Design

#### Sensor Fusion for Environmental Monitoring

**Weather Station:**
```
Sensors:
- Temperature: ±0.3°C accuracy
- Humidity: ±3% RH
- Pressure: ±1 hPa
- Wind speed: Anemometer (optical or cup)
- Wind direction: Vane or ultrasonic
- Rainfall: Tipping bucket (0.2 mm resolution)
- Solar radiation: Pyranometer (W/m²)

Data logging:
- Sampling: 1 minute intervals
- Averaging: 10 min for wind, hourly for rain
- Transmission: Cellular, LoRaWAN, satellite
```

**Smart Building HVAC:**
```
Zone Control:
- Temperature setpoint: 22°C ± 1°C
- Humidity target: 45-55% RH
- CO₂ limit: <1000 ppm (demand-controlled ventilation)
- Occupancy: PIR motion sensor, CO₂ trend

Control Strategy:
- PID temperature control
- Humidity: Modulate outdoor air intake, humidifier/dehumidifier
- Economizer mode: Free cooling when outdoor temperature suitable
- Predictive: Machine learning optimal start/stop times
```

**Industrial IoT Predictive Maintenance:**
```
Sensor Array on Pump/Motor:
- Vibration (MEMS accelerometer): Bearing wear, imbalance
- Temperature (RTD): Overheating, lubrication issues
- Pressure: Inlet/outlet performance
- Current: Motor load, efficiency

Analytics:
- Baseline: Normal operating signature
- Anomaly detection: Deviation from baseline
- Trend analysis: Gradual degradation
- Fault diagnosis: Pattern matching to known failure modes

Outcome:
- Schedule maintenance before failure
- Reduce downtime by 30-50%
- Extend equipment lifetime
```

#### Communication Protocols

**Industrial:**
- 4-20 mA: Analog standard, long-distance, noise-immune
- Modbus RTU/TCP: Widely adopted, simple
- HART: Digital + analog hybrid
- PROFIBUS/PROFINET: Fast, deterministic (PLC integration)
- EtherCAT: Ultra-low latency (<100 μs cycle)

**IoT:**
- I2C/SPI: Chip-to-chip communication
- UART: Simple serial, low-cost MCU interface
- LoRaWAN: Long-range (10 km), low-power (years on battery), low data rate
- NB-IoT: Cellular, wide coverage, higher cost
- Zigbee: Mesh network, home automation
- BLE: Bluetooth Low Energy, smartphones

**Selection Criteria:**
- Distance: I2C <1 m, RS-485 up to 1 km, LoRa 10+ km
- Data rate: I2C/SPI (Mbps), UART (kbps), LoRa (bps)
- Power: BLE/LoRa ultra-low for battery devices
- Cost: I2C/UART cheapest, cellular modem expensive

---

**References:**
- NIST ITS-90: *International Temperature Scale of 1990*
- ISO 16770: Plastics — Determination of environmental stress cracking (ESC) of polyethylene
- ISA Standards: Industrial Automation and Control Systems
- Omega Engineering: *Temperature Measurement Handbook*
- Beaty, H. W., & Kirtley, J. L. (2007). *Electric Motor Handbook*. McGraw-Hill.

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
