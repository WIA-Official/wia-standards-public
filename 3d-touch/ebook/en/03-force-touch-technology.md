# Chapter 3: Force Touch Technology

## Capacitive Force Sensing Principles and Implementation

### Introduction

Force Touch technology, pioneered by Apple in 2014-2015, revolutionized touch interfaces by adding a third dimension: force. Unlike traditional capacitive touch screens that only detect X-Y position, Force Touch measures how hard the user presses, enabling new interaction paradigms like Peek and Pop gestures, pressure-sensitive drawing, and context-dependent actions.

This chapter explores the physics, architecture, and implementation of capacitive force sensing, strain gauge arrays, and alternative force detection methods used in modern touch interfaces.

---

## Physics of Force Sensing

### Fundamental Principles

**Force vs. Pressure:**
- **Force (F)**: Measured in Newtons (N) or grams-force (gf), 1 gf ≈ 9.81 mN
- **Pressure (P)**: Force per unit area, P = F/A, measured in Pascals (Pa) or kPa
- **Typical finger force**: 0.5-5 N (50-500 gf) for normal touch interactions
- **Deep press force**: 3-8 N (300-800 gf) for intentional force gestures

**Contact Mechanics:**
When a finger presses a touch screen, the contact area increases with applied force:
- **Hertzian contact theory**: For elastic deformation
- **Contact area A ∝ F^(2/3)** for finger-glass interface
- **Typical finger contact**: 30-150 mm² depending on force and finger size

**Force Detection Methods:**
1. **Direct force measurement**: Strain gauges, piezoelectric sensors
2. **Indirect force estimation**: Capacitance change, optical deflection
3. **Hybrid approaches**: Combining multiple sensing modalities

---

## Capacitive Force Sensing Architecture

### Apple 3D Touch Design (iPhone 6s-Xs)

**Layer Stack (bottom to top):**
1. **Backlight unit** (LCD) or **OLED panel**
2. **Capacitive force sensor array** (etched ITO or metal traces)
3. **Cover glass** (chemically strengthened, 0.5-0.7mm thick)
4. **Capacitive touch sensor** (mutual capacitance grid)
5. **Oleophobic coating**

**Total stack thickness increase:** ~0.5mm vs. non-Force-Touch display

**Key Components:**

**1. Capacitive Sensor Array**
- **Design**: Grid of electrodes beneath the display
- **Pattern**: Orthogonal X-Y grid, 6×10 or 8×12 typical
- **Material**: ITO (Indium Tin Oxide) or metal mesh (copper/silver)
- **Electrode size**: 5-8mm pitch for good spatial resolution

**2. Deformable Dielectric**
- **Purpose**: Converts force to capacitance change
- **Material**: Compliant polymer with controlled elasticity
- **Thickness**: 50-150 μm typical
- **Compression**: 5-20 μm under force
- **Dielectric constant**: εᵣ = 2-4 typical

**3. Rigid Backing Plate**
- **Purpose**: Provides mechanical reference
- **Material**: Aluminum, steel, or rigid polymer
- **Thickness**: 0.3-0.5mm

### Capacitance-to-Force Conversion

**Parallel plate capacitor model:**

```
C = (ε₀ × εᵣ × A) / d

Where:
C = capacitance (Farads)
ε₀ = permittivity of free space = 8.854 × 10⁻¹² F/m
εᵣ = relative permittivity of dielectric material
A = electrode overlap area (m²)
d = gap distance between electrodes (m)
```

**Force application effect:**
- Applied force compresses dielectric layer
- Gap distance d decreases
- Capacitance C increases (C ∝ 1/d)
- Measure ΔC to determine force

**Typical values:**
- Baseline capacitance C₀: 20-50 pF per electrode pair
- Force-induced change ΔC: 0.5-5 pF for 300gf force
- Sensitivity: 10-20 fF per gram of force
- Detection limit: 0.1 gf with 1 fF resolution ADC

**Non-linearity challenges:**
- Contact area increases with force (A increases)
- Dielectric compression is non-linear (d vs. F non-linear)
- Temperature affects dielectric properties
- Aging changes material elasticity

**Linearization approach:**
```python
# Multi-point calibration
def linearize_force(raw_capacitance, cal_points):
    # Piecewise linear interpolation
    # cal_points = [(C1, F1), (C2, F2), ..., (Cn, Fn)]
    for i in range(len(cal_points) - 1):
        C_low, F_low = cal_points[i]
        C_high, F_high = cal_points[i + 1]
        if C_low <= raw_capacitance <= C_high:
            # Linear interpolation
            force = F_low + (raw_capacitance - C_low) * (F_high - F_low) / (C_high - C_low)
            return force
    return 0  # Out of range
```

---

## Strain Gauge Force Sensing

### Synaptics ClearForce Technology

**Principle:**
Strain gauges mounted beneath the display measure mechanical deformation when force is applied.

**Architecture:**
1. **Display assembly** (touchscreen, LCD/OLED)
2. **Strain gauge sensors** (4 corners or 6-point array)
3. **Rigid frame** for mounting
4. **Signal conditioning** (Wheatstone bridge, instrumentation amplifier)

**Strain Gauge Placement:**
- **4-corner design**: One sensor at each corner of display
- **6-sensor design**: Better spatial resolution, center detection
- **Edge-mounted**: Sensors on display perimeter

**Advantages over capacitive:**
- More linear force response
- Better temperature stability
- Lower cost (commodity strain gauges)
- Established technology

**Disadvantages:**
- Requires rigid display mounting
- Limited flexibility for curved or flexible displays
- Thicker stack (discrete sensors vs. integrated)
- Single-point force only (no multi-point)

### Strain Gauge Physics

**Resistance change under strain:**

```
ΔR/R = GF × ε

Where:
ΔR = resistance change (Ω)
R = nominal resistance (Ω)
GF = gauge factor (2.0-2.2 for metal foil gauges)
ε = strain (dimensionless, ΔL/L)
```

**Wheatstone bridge configuration:**
```
      +V
       |
    R1   R2 (strain gauge)
    |     |
Vout+     -Vout
    |     |
    R3   R4 (strain gauge)
       |
      GND

Output voltage: Vout = (V/4) × (ΔR/R) × GF
```

**Typical specifications:**
- Gauge factor: GF = 2.0-2.2
- Resistance: 120Ω, 350Ω, or 1000Ω
- Strain range: 0-2000 με (micro-strain)
- Force range: 0-10 N typical
- Linearity: ±0.5% of full scale

**Signal conditioning:**
- **Instrumentation amplifier**: Gain 100-500×
- **ADC resolution**: 16-24 bits for sub-gram resolution
- **Sampling rate**: 100-1000 Hz
- **Filtering**: 10-50 Hz low-pass to remove noise

---

## Piezoelectric Force Sensing

### Principles

**Piezoelectric effect:**
Certain materials (quartz, PZT ceramics, PVDF polymer) generate electric charge when mechanically stressed.

**Charge generation:**
```
Q = d × F

Where:
Q = charge (Coulombs)
d = piezoelectric charge constant (pC/N)
F = applied force (N)
```

**Common piezoelectric materials:**
- **Quartz (SiO₂)**: d = 2.3 pC/N, very stable, expensive
- **PZT (Lead Zirconate Titanate)**: d = 200-600 pC/N, high sensitivity
- **PVDF (Polyvinylidene Fluoride)**: d = 20-30 pC/N, flexible film

**Advantages:**
- Ultra-fast response (<1 ms)
- High sensitivity
- Wide dynamic range
- Self-powered (no DC bias needed)

**Disadvantages:**
- AC-coupled (only responds to changing force)
- Charge leakage (cannot measure static force)
- Temperature sensitive
- High output impedance (requires charge amplifier)

**Applications in touch:**
- Vibration/impact detection (complementary to capacitive touch)
- Haptic actuation (dual-use: sensing + actuation)
- Edge force sensing (detect squeeze gestures)

**Example: TDK PowerHap**
- Piezoelectric bending actuator
- Dual-use: Haptic feedback AND force sensing
- Response time: <1 ms (vs. 10-20 ms for capacitive)
- Applications: Laptop trackpads, automotive touchscreens

---

## Force Sensing Controller ICs

### Texas Instruments FDC2214 (Capacitance-to-Digital Converter)

**Key specifications:**
- 28-bit capacitance measurement resolution
- 4-channel simultaneous sampling
- Measurement range: 1 pF - 40 pF
- Resolution: <1 fF (femtofarad)
- Sampling rate: 13.3 kSPS per channel
- Interface: I2C (up to 400 kHz)

**Application in force sensing:**
- Measures capacitance change in force sensor array
- High resolution enables sub-gram force detection
- Multi-channel supports distributed sensor array
- Low power: 750 μA active, 5 μA shutdown

**Typical circuit:**
```
Force Sensor Array (4 channels)
    ↓↓↓↓
   FDC2214 (Capacitance Measurement)
    ↓
   I2C Interface
    ↓
  Microcontroller (Force Calculation)
```

### Cirrus Logic CS40L26 (Haptic Driver + Force Sensing)

**Integrated solution:**
- Haptic driver (LRA/ERM) + force sensing AFE
- 16-bit force measurement ADC
- Haptic waveform synthesis
- Closed-loop force feedback

**Key features:**
- Force sampling: 1 kHz
- Haptic latency: <5 ms from trigger to output
- Programmable force thresholds
- Interrupt generation on force events

**Use case: Smartphone force touch**
- Single IC handles both force detection and haptic response
- Reduces BOM cost and board space
- Synchronized force-haptic feedback

---

## Calibration Procedures

### Factory Calibration

**Multi-Point Calibration:**

**Equipment:**
- Calibrated force applicator (pneumatic or servo-controlled)
- Known force reference (0.1% accuracy typical)
- Environmental chamber (temperature control)

**Procedure:**
1. **Baseline measurement** (no force): Record C₀ or R₀
2. **Apply calibration forces**: F = [50g, 100g, 200g, 300g, 400g]
3. **Measure sensor response** at each force level
4. **Fit calibration curve** (polynomial or piecewise linear)
5. **Store calibration coefficients** in device EEPROM/flash

**Calibration equation (2nd-order polynomial):**
```
F = a₀ + a₁×C + a₂×C²

Where:
F = force (grams-force)
C = measured capacitance (pF)
a₀, a₁, a₂ = calibration coefficients
```

**Temperature Compensation:**
Repeat calibration at multiple temperatures [0°C, 25°C, 40°C, 60°C]

**Temperature-compensated force:**
```python
def compensated_force(raw_C, temp_C):
    # Base calibration at 25°C
    F_25 = a0 + a1 * raw_C + a2 * raw_C**2

    # Temperature coefficient (measured during calibration)
    temp_coeff = -0.002  # -0.2% per °C typical

    # Apply compensation
    F = F_25 * (1 + temp_coeff * (temp_C - 25))
    return F
```

### Runtime Calibration

**Auto-Zero:**
Periodically recalibrate baseline when no force is applied

**Detection criteria:**
- No touch detected for >500 ms
- Device stationary (accelerometer check)
- Screen off (no user interaction expected)

**Hysteresis Compensation:**
Force sensors exhibit hysteresis (different response for increasing vs. decreasing force)

**Approach:**
- Separate calibration curves for loading and unloading
- Track force direction (increasing/decreasing)
- Apply appropriate curve

```python
class ForceTracker:
    def __init__(self):
        self.last_force = 0
        self.direction = 0  # 1=increasing, -1=decreasing

    def get_force(self, raw_C):
        if raw_C > self.last_C:
            self.direction = 1
            force = self.loading_curve(raw_C)
        else:
            self.direction = -1
            force = self.unloading_curve(raw_C)

        self.last_C = raw_C
        return force
```

---

## Force Sensing Challenges and Solutions

### Challenge 1: Contact Area Variation

**Problem:**
Different users apply force with different finger sizes and postures, leading to varying contact areas.

**Impact:**
- Large contact area → higher capacitance even at low force
- Small contact area → lower capacitance even at high force
- Force estimation error: ±20-50% without compensation

**Solution: Normalized Force Metric**
```python
# Measure both capacitive touch area and force sensor response
touch_area = measure_touch_area()  # from capacitive touch controller
force_capacitance = measure_force_sensor()

# Normalize force by contact area
normalized_force = force_capacitance / sqrt(touch_area)

# Apply calibration with normalized metric
force_grams = calibration_curve(normalized_force)
```

### Challenge 2: Edge Effects

**Problem:**
Force applied near display edges produces different sensor response than center force.

**Solution: Spatial Calibration Map**
- Divide display into zones (e.g., 4×6 grid)
- Calibrate each zone independently
- Interpolate for positions between zones

```python
# Spatial calibration lookup
calibration_map = {
    'top_left': [a0_tl, a1_tl, a2_tl],
    'top_center': [a0_tc, a1_tc, a2_tc],
    # ... for each zone
}

def get_force(x, y, raw_C):
    zone = determine_zone(x, y)
    coeffs = calibration_map[zone]
    force = coeffs[0] + coeffs[1]*raw_C + coeffs[2]*raw_C**2
    return force
```

### Challenge 3: Multi-Touch Force Ambiguity

**Problem:**
Two simultaneous touches can create force sensor responses that are difficult to separate.

**Approaches:**

**1. Sensor Array Method (Apple 3D Touch)**
- High-density force sensor grid (6×10 electrodes)
- Spatial resolution ~10mm
- Can distinguish multiple force points if >20mm apart

**2. Combined Touch-Force Algorithm**
- Use capacitive touch position (high resolution, ~2mm)
- Map to nearest force sensor element
- Assign force based on proximity

```python
# Multi-touch force assignment
touches = get_touch_positions()  # [(x1,y1), (x2,y2), ...]
force_sensors = get_force_sensors()  # [F1, F2, ..., Fn]

for touch in touches:
    # Find nearest force sensor
    nearest_sensor = find_nearest(touch, force_sensors)
    # Assign force to this touch
    touch.force = nearest_sensor.force
```

---

## Alternative Force Sensing Techniques

### Optical Force Sensing

**Principle:**
Measure optical properties (reflectance, interference) that change with applied force.

**Approach 1: Imaging-based**
- Camera under display glass images finger contact
- Contact area increases with force
- Image processing estimates force

**Approach 2: Interferometry**
- Optical interference pattern changes with glass deflection
- High-resolution force measurement
- Complex optics, high cost

**Applications:**
- Research and specialized industrial interfaces
- Not widely adopted in consumer devices (cost, complexity)

### Acoustic Force Sensing

**Principle:**
Ultrasonic waves propagate differently through glass under stress.

**Method:**
- Ultrasonic transmitter excites glass
- Receiver measures wave propagation time
- Applied force changes acoustic path length
- Time-of-flight change → force estimate

**Advantages:**
- No additional layers in display stack
- Works with existing glass
- Can map force across entire surface

**Challenges:**
- Complex signal processing
- Multiple transmitters/receivers needed
- Sensitive to temperature, humidity

**Status:**
Experimental, research stage (not commercial)

---

## Force Touch in Smartwatches (Apple Watch)

### Apple Watch Force Touch Design

**Unique requirements:**
- Small display (38-45mm diagonal)
- Curved display (OLED on curved substrate)
- Water resistance (IPX8, 50m)
- Limited thickness budget (<11mm total)

**Architecture:**
- **Single-point force sensing**: Entire display acts as force sensor
- **No spatial resolution**: Cannot determine force location
- **Electrode ring**: Capacitive sensor around display perimeter
- **Compliant gasket**: Deformable seal provides force-to-capacitance coupling

**Implementation:**
- User presses anywhere on display
- Display assembly compresses gasket
- Capacitance change detected by ring electrode
- Taptic Engine provides haptic confirmation

**Gesture:**
- **Normal tap**: Standard touch interaction
- **Force press**: Activates context menu or additional options
- **Threshold**: ~150-200 gf typical

**Software integration:**
- watchOS APIs: `addGestureRecognizer(_:)` with `.force` type
- Apps use force press for secondary actions (e.g., change watch face)

---

## Force Touch Trackpads (MacBook)

### Design Evolution

**Pre-2015 (Mechanical Trackpad):**
- Physical click mechanism (diving board design)
- Mechanical switch at bottom
- Tactile click feedback from switch

**2015-Present (Force Touch Trackpad):**
- No mechanical click (solid-state)
- Four force sensors (strain gauges) at corners
- Taptic Engine (LRA) simulates click feel
- Variable force detection (light, medium, firm)

**Advantages:**
- Clickable anywhere (no diving board)
- Programmable click feel
- Force-variable actions (e.g., pressure-sensitive drawing)
- Better water/dust resistance (no mechanical parts)

**Force Sensing:**
- **Light press**: ~100-150 gf (normal click)
- **Firm press**: ~300-400 gf (Force Click, triggers secondary actions)

**Haptic Feedback:**
- Taptic Engine generates click sensation
- Waveform tuned to feel like mechanical click
- User-adjustable click firmness

**Gestures:**
- **Light click**: Select
- **Force Click**: Look up word, preview file, variable-speed scrubbing
- **Pressure sensitivity**: Drawing apps, video scrubbing

---

## Comparison: Force Sensing Technologies

| Technology | Accuracy | Latency | Cost | Thickness | Multi-Point | Temperature Stability |
|------------|----------|---------|------|-----------|-------------|-----------------------|
| **Capacitive Force** | ±2-5% | 8-15 ms | $$ | +0.5mm | Yes (grid) | Medium (compensation required) |
| **Strain Gauge** | ±0.5-2% | 5-10 ms | $ | +0.3mm | No (single point) | High (metal stable) |
| **Piezoelectric** | ±1-3% | <1 ms | $$$ | +0.2mm | Limited | Low (temp sensitive) |
| **Resistive (FSR)** | ±10-20% | 10-30 ms | $ | +0.5mm | Yes (array) | Medium (polymer drift) |
| **Optical** | ±5-10% | 20-50 ms | $$$$ | Varies | Yes (imaging) | Medium |

**Selection criteria:**
- **Smartphones**: Capacitive force (thin, multi-point) OR software Haptic Touch (cost)
- **Smartwatches**: Capacitive single-point (curved display, thin)
- **Trackpads**: Strain gauge (accuracy) OR piezoelectric (fast response)
- **Automotive**: Capacitive or piezoelectric (reliability, haptic integration)
- **Industrial**: Strain gauge (accuracy, robustness)

---

## Future Directions

### Distributed Force Sensing

**Concept:**
High-density force sensor array (1-2mm pitch) provides force mapping across entire display surface.

**Benefits:**
- Multi-finger force gestures
- Pressure-sensitive drawing with spatial resolution
- Detect grip force vs. intentional press

**Challenges:**
- Increased sensor channels (100+ vs. 4-10)
- Higher data rate and processing requirements
- Calibration complexity (each sensor individually calibrated)

### AI-Enhanced Force Detection

**Machine Learning Approach:**
Train neural network to predict force from multiple sensor modalities:
- Capacitive touch (position, area)
- Accelerometer (impact signature)
- Gyroscope (device tilt)
- Historical force patterns (user-specific)

**Benefits:**
- Improved accuracy without hardware cost
- User-adaptive force thresholds
- Anomaly detection (accidental vs. intentional press)

### Flexible Display Force Sensing

**Challenge:**
Foldable and rollable displays require flexible force sensors.

**Approaches:**
- PVDF piezoelectric film (flexible, stretchable)
- Printed strain gauge arrays on flexible substrate
- Optical fiber force sensing

**Status:**
Early research, not yet in commercial foldable devices.

---

## Conclusion

Force Touch technology adds a valuable third dimension to touch interfaces, enabling richer interactions and novel user experiences. While smartphone adoption declined due to cost and user discovery challenges, the underlying technology continues to evolve in trackpads, automotive interfaces, gaming, and industrial applications.

**Key technical achievements:**
1. Sub-gram force resolution with capacitive sensing
2. <10ms latency for responsive force detection
3. Multi-point force sensing via electrode arrays
4. Robust calibration for varied users and environmental conditions

**Next Chapter Preview:** Chapter 4 explores Haptic Touch systems, covering LRA, ERM, and piezoelectric actuators, waveform synthesis, and tactile feedback design patterns.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
