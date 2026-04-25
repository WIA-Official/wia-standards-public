# Chapter 3: MEMS Sensors - Micro-Electro-Mechanical Systems

## Introduction to MEMS Technology

MEMS (Micro-Electro-Mechanical Systems) represents one of the most transformative technologies in modern sensor design. By integrating mechanical elements, sensors, actuators, and electronics on a common silicon substrate using microfabrication techniques, MEMS enables unprecedented miniaturization, performance, and cost-effectiveness.

### What Makes MEMS Special?

**Size Scale:**
- Feature sizes: 1-100 micrometers (μm)
- Device dimensions: 0.5-10 millimeters
- Comparison: Human hair diameter ≈ 75 μm

**Benefits of MEMS:**
1. **Miniaturization**: 1000x smaller than conventional sensors
2. **Mass Production**: Batch fabrication like ICs
3. **Low Cost**: Economies of scale
4. **Low Power**: Small mass, minimal actuation force
5. **High Performance**: Precision lithography, controlled material properties
6. **Integration**: Combine sensing + signal processing on-chip

### MEMS Fabrication Technologies

#### Bulk Micromachining

**Process Description:**
Selective etching of the silicon substrate to create mechanical structures.

**Techniques:**
1. **Wet Etching:**
   - Anisotropic: KOH, TMAH (creates crystallographic-plane-dependent structures)
   - Isotropic: HF, HNA (etches equally in all directions)

2. **Dry Etching:**
   - DRIE (Deep Reactive Ion Etching): Bosch process
   - High aspect ratio structures (>20:1)
   - Vertical sidewalls

**Applications:**
- Pressure sensors (diaphragms)
- Accelerometers (proof mass with springs)
- Inkjet nozzles

**Example Process Flow:**
```
1. Start: Silicon wafer (100 or 110 orientation)
2. Thermal oxidation: Grow SiO2 mask layer
3. Photolithography: Pattern etch windows
4. KOH wet etch: Create suspended structures
5. Remove oxide mask
6. Bond to second wafer (cap/base)
```

**Advantages:**
- Simple, well-established processes
- Large structures possible
- Good for pressure sensors

**Limitations:**
- Geometry limited by crystallographic planes
- Thick substrates reduce sensitivity
- Difficult to make complex 3D structures

#### Surface Micromachining

**Process Description:**
Sequential deposition and patterning of thin films on the substrate surface. Sacrificial layers are removed to release movable structures.

**Key Materials:**
- Structural: Polysilicon, silicon nitride, metals
- Sacrificial: Silicon dioxide, photoresist
- Etchants: HF (removes SiO2), oxygen plasma (removes photoresist)

**Typical Process:**
```
1. Substrate: Silicon wafer
2. Deposit sacrificial layer: 1-2 μm SiO2 (LPCVD or PECVD)
3. Pattern anchors: Photolithography + etch
4. Deposit structural layer: 2-5 μm polysilicon (LPCVD)
5. Pattern structural features: Photolithography + RIE
6. Release: HF vapor etch removes SiO2
7. Dry (critical): Supercritical CO2 or freeze-drying
```

**Applications:**
- Accelerometers (comb-drive structures)
- Gyroscopes (vibrating structures)
- Resonators
- Micromirrors (DMD, MEMS scanners)

**Advantages:**
- Complex 3D structures
- Precise dimensional control (nm-level)
- CMOS compatibility
- Multiple structural layers possible

**Challenges:**
- Stiction during release (structural collapse)
- Stress in deposited films causes warping
- Limited thickness (<10 μm typically)

#### SOI (Silicon-on-Insulator) MEMS

**Structure:**
- Device layer: Single-crystal silicon (1-100 μm)
- Buried oxide (BOX): SiO2 insulator (0.5-3 μm)
- Handle wafer: Silicon substrate (500-700 μm)

**Advantages Over Polysilicon:**
- Superior mechanical properties (single-crystal vs. polycrystalline)
- Lower internal stress
- Uniform thickness
- Higher Q-factor for resonators

**Process:**
```
1. Start: SOI wafer
2. Pattern device layer: Define mechanical structures
3. DRIE etch: Through device layer, stop on BOX
4. Backside process: Thin or remove handle wafer
5. BOX etch: Release structures (HF)
```

**Applications:**
- High-performance gyroscopes
- RF MEMS switches and resonators
- Pressure sensors
- Optical MEMS

**Cost Consideration:**
- SOI wafers: 3-5x more expensive than bulk silicon
- Justified for performance-critical applications

#### LIGA Process

**LIGA: Lithographie, Galvanoformung, Abformung** (Lithography, Electroplating, Molding)

**Process:**
1. X-ray lithography with synchrotron radiation
2. Deep, high-aspect-ratio photoresist structures (>100:1)
3. Electroplating: Nickel, gold, copper
4. Molding: Create plastic replicas

**Features:**
- Aspect ratios: >100:1
- Feature height: Up to 1000 μm
- Sidewall roughness: <50 nm

**Applications:**
- Microfluidic pumps and valves
- Micro-gears and actuators
- Inertial sensors (high proof mass)

**Limitations:**
- Expensive (synchrotron access)
- Low throughput
- Limited to niche applications

### MEMS Accelerometers

Accelerometers measure linear acceleration in one or more axes. MEMS accelerometers dominate the market due to size, cost, and performance.

#### Operating Principle

**Fundamental Equation:**
```
F = m × a

Where:
F = Force on proof mass
m = Proof mass (seismic mass)
a = Acceleration
```

**Spring-Mass-Damper Model:**
```
m × d²x/dt² + c × dx/dt + k × x = m × a

Where:
m = Proof mass
c = Damping coefficient
k = Spring constant
x = Displacement
a = External acceleration
```

**Resonant Frequency:**
```
f₀ = (1 / 2π) × √(k / m)

Typical values:
- Consumer MEMS: 1-5 kHz
- Automotive: 500 Hz - 2 kHz
- High-g shock: 10-50 kHz
```

**Sensitivity:**
```
S = m / k   (displacement per g)

Trade-off:
- High sensitivity: Large mass, soft spring (low f₀, fragile)
- High bandwidth: Small mass, stiff spring (low sensitivity)
```

#### Sensing Mechanisms

**1. Capacitive Sensing (Most Common)**

**Parallel Plate Capacitance:**
```
C = ε₀ × ε_r × A / d

Where:
ε₀ = Permittivity of free space (8.854 pF/m)
ε_r = Relative permittivity (air ≈ 1)
A = Overlap area
d = Gap distance
```

**Differential Capacitance:**
- Fixed electrodes on both sides of proof mass
- Acceleration causes asymmetric gap change
- Differential measurement rejects common-mode noise

**Capacitance Change:**
```
ΔC / C₀ = x / d₀   (for small displacements)

Typical values:
- C₀ = 100 fF - 10 pF
- ΔC = 1-100 aF per g
- d₀ = 1-5 μm
```

**Advantages:**
- Low power consumption (<10 μW)
- Temperature stable
- Simple readout circuits
- Low noise

**Challenges:**
- Small signal (aF range)
- Parasitic capacitance
- Requires ASIC with low-noise amplifier

**2. Piezoresistive Sensing**

**Principle:**
- Resistivity changes under mechanical stress
- Piezoresistors placed on flexures
- Wheatstone bridge configuration

**Gauge Factor:**
```
GF = (ΔR / R) / ε

Where:
ΔR/R = Relative resistance change
ε = Strain

Silicon piezoresistors:
- Longitudinal: GF ≈ +100 to +200
- Transverse: GF ≈ -50 to -100
```

**Sensitivity:**
```
Output voltage = V_exc × GF × ε / 4

For 5V excitation, GF=100, ε=10⁻⁶:
V_out ≈ 125 μV
```

**Advantages:**
- Simple fabrication (ion implantation)
- Robust (no moving plates)
- Wide measurement range

**Disadvantages:**
- Temperature dependence (requires compensation)
- Higher power (mW range)
- Lower sensitivity than capacitive

**3. Piezoelectric Sensing**

**Principle:**
- Certain crystals generate charge under stress
- PZT, AlN, ZnO materials

**Charge Generation:**
```
Q = d × F

Where:
Q = Charge
d = Piezoelectric coefficient (pC/N)
F = Force
```

**Advantages:**
- Self-generating (no power needed for sensing)
- High sensitivity
- Wide frequency response

**Disadvantages:**
- Cannot measure DC (static) acceleration
- Charge leakage over time
- Requires charge amplifier
- Mainly used for vibration sensing

**4. Thermal (Heat Transfer) Sensing**

**Principle:**
- Heated bubble of gas
- Acceleration causes convective asymmetry
- Temperature differential measured by thermopiles

**Advantages:**
- No mechanical suspension (ultra-robust)
- Survive >50,000 g shocks
- Very low cost

**Disadvantages:**
- High power consumption (mW)
- Slow response time (ms)
- Sensitive to ambient temperature changes
- Limited to low-frequency applications

#### Commercial MEMS Accelerometer Examples

**Bosch BMA456:**
- Type: 3-axis capacitive
- Range: ±2g, ±4g, ±8g, ±16g (selectable)
- Sensitivity: 16,384 LSB/g @ ±2g
- Noise density: 120 μg/√Hz
- Current: 150 μA (normal mode)
- Size: 1.5 × 1.5 × 0.72 mm³
- Interface: I2C, SPI
- Features: Embedded interrupt engines, tap detection

**STMicroelectronics LIS2DW12:**
- Type: 3-axis capacitive
- Range: ±2g, ±4g, ±8g, ±16g
- Resolution: 14-bit
- Noise: 90 μg/√Hz
- Current: 50 μA @ 50 Hz
- Ultra-low power mode: 380 nA
- Features: FIFO, embedded finite state machine

**Analog Devices ADXL355:**
- Type: 3-axis capacitive
- Range: ±2g, ±4g, ±8g
- Resolution: 20-bit
- Noise density: 25 μg/√Hz (excellent!)
- Bias instability: 10 μg
- Temperature range: -40°C to +125°C
- Application: Industrial, high-accuracy

**Murata SCA3300:**
- Type: 3-axis capacitive
- Range: ±1.5g, ±3g, ±6g
- Bandwidth: 900 Hz
- Noise: 70 μg/√Hz
- Features: ASIL-D automotive safety, SPI interface
- Package: Hermetically sealed ceramic

### MEMS Gyroscopes

Gyroscopes measure angular velocity (rate of rotation). MEMS gyroscopes use the Coriolis effect in vibrating structures.

#### Operating Principle

**Coriolis Force:**
```
F_c = 2 × m × v × Ω

Where:
F_c = Coriolis force
m = Proof mass
v = Velocity of vibrating mass (drive axis)
Ω = Angular velocity (rate of rotation)
```

**Operating Mode:**
1. **Drive Mode:**
   - Proof mass oscillates at resonant frequency (drive axis)
   - Typical frequency: 5-30 kHz
   - Amplitude: 1-10 μm

2. **Sense Mode:**
   - Rotation causes Coriolis force perpendicular to drive motion
   - Secondary vibration in sense axis
   - Amplitude proportional to rotation rate

**Frequency Matching:**
- Ideal: Drive and sense resonant frequencies matched (Δf < 1 Hz)
- Reality: Manufacturing variations cause mismatch
- Solution: Electronic tuning or mechanical trimming

**Quality Factor (Q):**
```
Q = f₀ / Δf = (Energy stored) / (Energy dissipated per cycle)

Typical values:
- Atmospheric pressure: Q = 10-100 (air damping)
- Vacuum package: Q = 1000-100,000
```

**Higher Q benefits:**
- Increased sensitivity
- Better signal-to-noise ratio
- Lower power (sustain resonance)

**Trade-offs:**
- High Q = narrow bandwidth, slow settling time
- Vacuum packaging adds cost and complexity

#### Gyroscope Architectures

**1. Tuning Fork Gyroscope**

**Structure:**
- Two proof masses
- Vibrate in anti-phase (opposite directions)
- Common-mode vibration rejection

**Advantages:**
- Cancels linear acceleration (differential sensing)
- Rejects external vibrations
- Good for single-axis

**Example:** STMicroelectronics L3GD20H
- Range: ±245, ±500, ±2000 dps (degrees per second)
- Sensitivity: 8.75 mdps/digit @ ±245 dps
- Zero-rate level: ±10 dps
- Current: 6 mA

**2. Ring/Disk Gyroscope**

**Structure:**
- Circular or ring-shaped resonator
- Flexural vibration modes
- Can sense rotation in-plane

**Advantages:**
- Symmetric structure
- Multiple sensing axes possible
- High Q-factor achievable

**Application:**
- Research and high-end inertial navigation
- Hemispherical resonator gyro (HRG)

**3. Quad Mass Gyroscope**

**Structure:**
- Four coupled proof masses
- Complex mode shapes
- Differential sensing on multiple axes

**Advantages:**
- Mechanical robustness
- Excellent vibration rejection
- Suitable for 3-axis integration

**Example:** TDK InvenSense ICM-42688-P
- 3-axis gyroscope + 3-axis accelerometer
- Gyro range: ±2000 dps
- Gyro noise: 0.004 dps/√Hz (industry-leading)
- Bias instability: 2 dps
- Synchronous sampling for sensor fusion

#### Gyroscope Performance Metrics

**1. Sensitivity (Scale Factor)**
```
S = Output / (Angular rate)
Units: mV/(°/s) or LSB/(°/s)

Example: 16.4 LSB/(°/s) means:
100 °/s rotation → 1640 LSB digital output
```

**2. Zero-Rate Output (Bias)**
- Output when no rotation
- Caused by: stress, asymmetry, electronics
- Specified: ±10 dps typical for consumer MEMS

**3. Bias Instability**
- Variation of bias over time
- Measured with Allan variance
- Units: °/s or °/hr
- Consumer MEMS: 5-20 dps
- Tactical grade: 0.1-10 °/hr
- Navigation grade: <0.01 °/hr

**4. Angular Random Walk (ARW)**
```
ARW = Noise density × √(Integration time)
Units: °/√hr

Typical values:
- Consumer MEMS: 0.1 - 1 °/√hr
- Automotive: 0.01 - 0.1 °/√hr
- Tactical: <0.01 °/√hr
```

**5. Bandwidth**
- Frequency range of measurable rotation
- Limited by: mechanical resonance, electronics
- Consumer: 200-400 Hz
- High-performance: Up to 1 kHz

**6. Cross-Axis Sensitivity**
- Sensitivity to rotation on orthogonal axes
- Ideally: 0%
- Typical: <2%
- Caused by: fabrication imperfections, packaging stress

### MEMS Magnetometers

Magnetometers measure magnetic field strength and direction. Used for electronic compass applications.

#### Operating Principles

**1. Lorentz Force Magnetometer**

**Principle:**
```
F = q × (v × B)

For a current-carrying conductor:
F = I × L × B

Where:
I = Current
L = Length of conductor
B = Magnetic field
```

**Implementation:**
- MEMS cantilever carries AC current
- Magnetic field causes Lorentz force
- Cantilever deflection detected (capacitive or piezoresistive)

**Sensitivity:**
- Consumer MEMS: 0.01-0.1 μT resolution
- Earth's magnetic field: ≈25-65 μT

**2. Hall Effect Magnetometer**

**Principle:**
- Current through conductor in magnetic field
- Charge carriers deflected (Hall effect)
- Voltage proportional to B-field

**Hall Voltage:**
```
V_H = (I × B) / (n × q × t)

Where:
n = Carrier concentration
q = Electron charge
t = Thickness
```

**Advantages:**
- Solid-state (no moving parts)
- Can measure DC fields
- Linear response

**Disadvantages:**
- Lower sensitivity than Lorentz force
- Temperature drift
- Offset voltage

**3. Resonant Magnetometer**

**Principle:**
- MEMS resonator under magnetic field
- Lorentz force changes resonant frequency
- Frequency shift proportional to B-field

**Advantages:**
- Frequency output (noise immunity)
- High sensitivity achievable

**Challenges:**
- Requires vacuum packaging
- Complex readout electronics

#### Commercial Examples

**Bosch BMM350:**
- 3-axis magnetometer
- Range: ±1300 μT (Earth's field ±65 μT)
- Noise: 0.3 μT RMS
- Current: 230 μA active, 3 μA suspend
- Interface: I2C
- Size: 1.45 × 1.05 × 0.52 mm³

**STMicroelectronics LIS2MDL:**
- 3-axis magnetometer
- Range: ±50 gauss (±5000 μT)
- Sensitivity: 1.5 mGauss/LSB
- Noise: 3 mGauss RMS
- Self-test capability

**Application: Electronic Compass**
- Combines magnetometer + accelerometer
- Tilt compensation using accelerometer
- Calculate heading (azimuth angle)
- Accuracy: ±2° typical, ±5° worst-case

**Calibration Challenges:**
- Hard-iron effects: Permanent magnets in device (smartphone speakers)
- Soft-iron effects: Ferromagnetic materials distort field
- Requires 3D calibration (rotate device in all orientations)

### MEMS Pressure Sensors

#### Sensing Mechanism

**Piezoresistive Pressure Sensors:**

**Structure:**
- Thin silicon diaphragm (10-100 μm)
- Piezoresistors implanted or diffused on diaphragm
- Wheatstone bridge configuration

**Stress Distribution:**
- Maximum tensile stress at diaphragm edges
- Maximum compressive stress at center
- Piezoresistors positioned for maximum sensitivity

**Pressure-to-Stress:**
```
σ = α × P × (a/h)²

Where:
σ = Stress
P = Pressure
a = Diaphragm radius
h = Diaphragm thickness
α = Geometry factor (≈0.2 for square, ≈0.24 for circular)
```

**Sensitivity:**
```
ΔR/R = π × σ

Where:
π = Piezoresistive coefficient (≈100 × 10⁻¹¹ Pa⁻¹ for Si)
```

#### Pressure Sensor Types

**1. Absolute Pressure**
- Reference: Vacuum cavity
- Application: Altitude measurement, weather stations
- Range: 30-110 kPa (for barometric pressure)

**Bosch BMP390:**
- Range: 300-1250 hPa (30-125 kPa)
- Accuracy: ±0.5 hPa (±0.5 mbar)
- Altitude resolution: ±0.08 m (8 cm!)
- RMS noise: 0.03 Pa
- Current: 3.2 μA @ 1 Hz, 714 μA @ 100 Hz
- Temperature sensor integrated

**Altitude Calculation:**
```
h = 44330 × [1 - (P / P₀)^(1/5.255)]

Where:
h = Altitude (meters)
P = Measured pressure
P₀ = Sea-level pressure (101325 Pa)
```

**2. Gauge Pressure**
- Reference: Atmospheric pressure (vented)
- Application: Blood pressure, tire pressure
- Range: 0-500 kPa typical

**3. Differential Pressure**
- Measures difference between two ports
- Application: Flow measurement, HVAC, medical ventilators
- Range: ±100 Pa to ±1000 kPa

**TE Connectivity MS4525DO:**
- Differential: ±1 psi (6.9 kPa)
- Accuracy: ±0.25% full scale
- Digital I2C output
- Application: Drone airspeed sensors

### MEMS Microphones

#### Acoustic Sensing

**Structure:**
- Flexible diaphragm (silicon or polymer)
- Back-plate electrode (capacitive sensing)
- Acoustic port for sound entry

**Operating Principle:**
- Sound pressure causes diaphragm deflection
- Capacitance change between diaphragm and back-plate
- ASIC amplifies and converts to analog or digital output

**Sensitivity:**
```
S = ΔC / ΔP

Typical: -38 dBV @ 94 dB SPL (1 Pa)
Means: 12.6 mV RMS output for 1 Pa acoustic pressure
```

**Signal-to-Noise Ratio (SNR):**
```
SNR = 94 dB SPL - Noise floor

Excellent: >68 dB (noise floor 26 dB SPL)
Good: 64-68 dB
Acceptable: 60-64 dB
```

**Acoustic Overload Point (AOP):**
- Maximum SPL before distortion >10%
- Typical: 120-135 dB SPL
- Equivalent to jet engine at 100 m distance

#### Commercial MEMS Microphones

**Knowles SPH0655:**
- Analog output
- Sensitivity: -38 dBV
- SNR: 64 dB (A-weighted)
- Frequency range: 100 Hz - 10 kHz
- Size: 3.76 × 2.95 × 1.0 mm³

**TDK InvenSense ICS-43434:**
- Digital I2S output
- SNR: 67 dB (A-weighted)
- AOP: 123 dB SPL
- Power: 620 μA active
- Application: Smartphone voice calls, recording

**Infineon IM69D130:**
- High-performance digital
- SNR: 69 dB (industry-leading)
- Phase accuracy: ±3° (for beamforming arrays)
- Flat frequency response: ±1 dB, 20 Hz - 20 kHz
- Application: Premium smartphones, conferencing systems

**Microphone Array Beamforming:**
- Multiple microphones (4-8 in smartphone)
- Digital signal processing
- Directional audio capture
- Noise suppression from specific directions

### Challenges and Future Directions

#### Manufacturing Challenges

**1. Process Variations:**
- Dimensional tolerances: ±0.1 μm
- Film stress: Causes frequency shift, sensitivity variation
- Doping uniformity: Affects piezoresistive sensors

**Solutions:**
- Wafer-level compensation
- Laser trimming
- Electronic calibration and trimming

**2. Packaging-Induced Stress:**
- Molding compound shrinkage
- CTE (thermal expansion) mismatch
- Solder reflow stress

**Solutions:**
- Stress-isolation designs
- Soft die attach materials
- Post-package calibration

**3. Particle Contamination:**
- Critical for MEMS with small gaps (<2 μm)
- Causes stiction or blocked motion
- Requires cleanroom fabrication (Class 100-1000)

#### Future MEMS Innovations

**1. Piezoelectric MEMS (PiezoMEMS):**
- AlN, ScAlN materials
- Self-powering capability
- Higher sensitivity than capacitive

**2. Resonant Sensors:**
- Output is frequency (noise-immune digital signal)
- Applications: Pressure, gas, mass detection
- Q-factors >10,000 in vacuum

**3. Multi-Axis Integration:**
- 10-DOF sensors: 3-axis accel + 3-axis gyro + 3-axis mag + pressure
- Single package <3 mm³
- Shared electronics, calibration

**4. AI-Enhanced MEMS:**
- On-chip machine learning
- Pattern recognition (fall detection, gesture)
- Adaptive calibration

**5. Flexible MEMS:**
- Printed on polymer substrates
- Conformable to curved surfaces
- Applications: Wearables, robot skin

---

**References:**
- Senturia, S. D. (2001). *Microsystem Design*. Springer.
- Madou, M. J. (2011). *Fundamentals of Microfabrication*. CRC Press.
- Bosch Sensortec Technical Documentation
- STMicroelectronics MEMS Design Guides
- IEEE Sensors Journal (selected papers)

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
