# Chapter 8: Display Assembly and System Integration

## From Individual LEDs to Complete Display

The journey from millions of microscopic LED chips to a functioning display involves sophisticated integration of multiple subsystems: TFT backplanes for individual pixel control, driver electronics for signal processing, power management, thermal systems, and optical enhancements. This chapter explores how these components come together to create commercial MicroLED displays.

## TFT Backplane Technologies

### TFT Role and Requirements

The thin-film transistor (TFT) backplane serves as the addressing and control substrate for MicroLED displays:

**Functions**:
- Individual pixel selection and control
- Current regulation for each LED
- Storage of image data (in some architectures)
- Compensation for LED variations

**Requirements**:
- Pixel pitch matching LED layout (10 μm to 2 mm)
- Current drive capability: 1 μA to 10 mA per pixel
- Switching speed: <100 μs for 120 Hz refresh
- Uniformity: <5% transistor variation
- Reliability: >50,000 hours operation

### TFT Technologies for MicroLED

**Low-Temperature Polysilicon (LTPS)**:

**Properties**:
- Electron mobility: 100-200 cm²/V·s
- Process temperature: <600°C (glass-compatible)
- Current drive capability: Excellent
- Uniformity: Good (±5-10%)

**Advantages**:
- High mobility enables high current drive
- Compact pixel circuits (higher aperture ratio)
- Proven in OLED smartphones
- Can integrate drivers on glass (GOA - Gate-on-Array)

**Challenges**:
- Limited to smaller panels (<15" for LTPS on glass)
- Higher cost than a-Si
- Laser crystallization adds process complexity

**Applications**: Smartphones, tablets, smartwatches, laptop displays

**Oxide TFT (IGZO - Indium Gallium Zinc Oxide)**:

**Properties**:
- Electron mobility: 10-50 cm²/V·s
- Process temperature: <400°C
- Uniformity: Excellent (±2-5%)
- Scalable to large areas

**Advantages**:
- Uniform over large areas (TVs, signage)
- Lower off-state leakage than a-Si
- Better stability than a-Si
- Compatible with large glass fabs (Gen 8+)

**Challenges**:
- Lower mobility than LTPS
- Requires larger transistors for same drive current
- Threshold voltage shifts under bias stress (improving)

**Applications**: Large TVs, monitors, commercial displays

**LTPO (Low-Temperature Polycrystalline Oxide)**:

**Concept**: Hybrid combining LTPS (for drive TFT) and oxide (for switching TFT)

**Advantages**:
- High current capability from LTPS
- Low leakage from oxide (enables VRR, variable refresh rate)
- Power savings in low refresh rate modes
- Used in premium smartphone OLEDs (Apple, Samsung)

**Challenges**:
- More complex fabrication
- Higher cost
- Limited suppliers

**Applications**: Premium smartphones, smartwatches with always-on displays

**CMOS Silicon Backplane**:

**Concept**: Use silicon wafer with integrated CMOS circuits

**Advantages**:
- Highest performance (high speed, high current)
- Can integrate complex circuitry (ADC, DAC, memory, processors)
- Excellent uniformity
- Ideal for micro-displays

**Challenges**:
- Limited wafer sizes (12-inch maximum)
- Much higher cost than glass-based TFT
- Cannot scale to large displays
- Requires wafer bonding or hybrid integration with LEDs

**Applications**: AR/VR microdisplays (<1" diagonal), near-eye displays

### Pixel Circuit Designs

**2T1C (2 Transistors, 1 Capacitor)**:

**Circuit**:
- T1: Select transistor
- T2: Drive transistor
- C1: Storage capacitor

**Operation**:
1. Select line activated, data voltage written to gate of T2
2. Capacitor stores voltage
3. T2 regulates current through LED based on stored voltage

**Advantages**:
- Simple, compact
- Proven in OLED

**Challenges**:
- Sensitive to T2 threshold voltage variation
- Limited compensation for LED variations

**Use**: High-uniformity TFT processes (LTPS, CMOS)

**3T1C, 4T2C, and More Complex Circuits**:

**Added Functions**:
- Threshold voltage compensation
- LED voltage drop compensation
- Current programming (more precise than voltage programming)
- Emission control (separate from addressing)

**Example 4T2C Circuit**:
- T1: Select
- T2: Drive
- T3: Compensation sensing
- T4: Emission control
- C1: Storage
- C2: Compensation storage

**Advantages**:
- Better uniformity (compensates for TFT and LED variations)
- Enables lower-uniformity TFT technologies (cost savings)

**Disadvantages**:
- Larger pixel area
- Lower aperture ratio (fill factor)
- More complex driving schemes

**Use**: Large-area displays where TFT uniformity challenging

**Current Mirror and Current Source Circuits**:

**Concept**: Program a reference current, mirror to each pixel

**Advantages**:
- Direct current control (avoids voltage-to-current conversion variations)
- Better matching across pixels

**Challenges**:
- Requires high-mobility TFTs
- More complex circuitry

### TFT Fabrication Process

**Standard TFT Process Flow** (LTPS example):

1. **Glass Cleaning**: Substrate preparation (Gen 6 glass, 1500×1850 mm typical)
2. **Buffer Layer**: SiO₂ deposition by PECVD
3. **Amorphous Si Deposition**: a-Si layer 30-50 nm
4. **Laser Crystallization**: Excimer laser (XeCl, 308 nm) crystallizes a-Si to poly-Si
5. **Active Layer Patterning**: Define transistor channel regions
6. **Gate Insulator**: SiO₂ or SiN 50-150 nm
7. **Gate Metal**: Mo, MoW, or Al alloys, 200-500 nm, patterned
8. **Doping**: Ion implantation or plasma doping for source/drain regions
9. **Interlayer Dielectric (ILD)**: SiO₂ 200-500 nm
10. **Contact Vias**: Open holes to source/drain regions
11. **Source/Drain Metal**: Al or Cu 300-500 nm, patterned
12. **Passivation**: SiN 200-400 nm protective layer
13. **Pixel Electrode Opening**: Define LED landing pads
14. **Pixel Electrode**: ITO or Al, patterned for LED bonding sites

**Process Time**: 10-15 days for complete TFT array

**Yield Challenges**:
- Particles and defects can short gate lines or data lines
- Uniformity of laser crystallization
- Alignment across multiple lithography steps (5-7 masks)

## Display Driver Integration

### Driver IC Architecture

**Functions of Driver ICs**:
- Convert digital image data to analog voltages/currents
- Multiplex signals to thousands of rows and columns
- Implement grayscale control (PWM or amplitude modulation)
- Timing control and synchronization
- Store calibration data for uniformity compensation

**Gate Driver (Row Driver)**:
- Sequentially activates each row of pixels
- Output channels: 1,000-4,000 per chip
- Voltage levels: -7V to +15V typical
- Shift register advances row selection

**Source Driver (Column Driver)**:
- Provides data voltages/currents to each column
- Output channels: 1,000-3,000 per chip
- DAC resolution: 8-10 bits per color channel
- Sample-and-hold output buffers

**Timing Controller (TCON)**:
- Interfaces with system (LVDS, eDP, MIPI, HDMI)
- Generates timing signals for gate and source drivers
- Image processing (scaling, color correction, etc.)
- Stores and applies calibration data

### Driver Mounting Technologies

**Chip-on-Glass (COG)**:
- Driver ICs bonded directly to TFT glass edge
- Anisotropic conductive film (ACF) bonding
- Compact, low-cost
- Standard for smartphones, tablets

**Chip-on-Film (COF)**:
- Driver ICs mounted on flexible PCB (TCP - Tape Carrier Package)
- TCP bonded to glass with ACF
- Allows drivers to wrap behind display (thinner bezel)
- Standard for larger displays, TVs

**Chip-on-Board (COB)**:
- Glass bonded to separate PCB
- Drivers on PCB connected via ribbon cables
- Used for modular displays (The Wall)
- Easier repair and replacement

**Integrated Drivers (GOA - Gate-on-Array)**:
- Gate drivers fabricated in TFT process on glass
- Reduces component count and cost
- Eliminates one set of IC attachments
- Common in large TVs

### Grayscale Control Methods

**Pulse Width Modulation (PWM)**:

**Principle**: Vary duty cycle of LED on-time

**Advantages**:
- Excellent linearity
- Wide dynamic range (10-16 bits achievable)
- Independent of LED voltage variations

**Challenges**:
- Requires high-frequency switching (kHz range)
- Can cause flicker if frequency too low
- More complex drive circuitry

**Typical**: 60 Hz frame rate × 256 grayscales = 15.36 kHz PWM frequency minimum

**Amplitude Modulation (AM)**:

**Principle**: Vary current/voltage amplitude

**Advantages**:
- Simpler drive circuitry
- No high-frequency switching (lower EMI)

**Challenges**:
- Sensitivity to TFT and LED variations
- Requires compensation circuits
- Limited dynamic range (8-10 bits typical)

**Hybrid PWM + AM**:
- Coarse control with amplitude (e.g., 4 bits)
- Fine control with PWM (e.g., 4-8 bits)
- Combines advantages of both methods
- Increasingly common

## Optical Enhancements

### Light Extraction Improvements

**Microlens Arrays**:

**Concept**: Lens array over each pixel to collimate and direct light

**Benefits**:
- Increases brightness in forward direction by 20-40%
- Can shape emission pattern (narrower viewing angle if desired)
- Improves contrast by reducing scatter

**Fabrication**:
- Photoresist reflow to form lens profiles
- UV-curable polymer stamping
- Directly integrated into cover glass

**Challenges**:
- Precise alignment to pixels (<5 μm)
- Adds thickness and cost
- Less effective for wide viewing angle requirements

**Scattering Films**:

**Concept**: Diffuser films to homogenize emission

**Use Cases**:
- Large-format displays where individual LED visibility is issue
- Transparent displays to scatter light

**Trade-offs**:
- Improves uniformity
- Reduces peak brightness and contrast

### Anti-Reflective Coatings

**Purpose**: Reduce ambient light reflections for better contrast

**Technologies**:
- Multi-layer optical coatings on cover glass
- Moth-eye nanostructures (sub-wavelength patterns)
- Circular polarizers (like in OLED)

**Performance**:
- Standard AR coating: ~1-2% reflection
- Advanced AR: <0.5% reflection
- Moth-eye: <0.2% reflection

**Impact**:
- Critical for displays in bright environments
- Improves perceived contrast ratio by 5-10×

## Thermal Management

### Heat Generation in MicroLED Displays

**Power Dissipation**:

For 65" 4K display at 500 nits:
- LED efficiency: ~40% (60% lost as heat)
- Power consumption: ~150W
- Heat dissipation: ~90W

**Heat Concentration**:
- Small chips generate high heat flux (W/cm²)
- LED efficiency and lifetime decrease with temperature
- Target junction temperature: <85°C

### Cooling Strategies

**Passive Cooling**:

**Heatsinks and Spreaders**:
- Aluminum or copper back-plates
- Graphite sheets (high in-plane conductivity)
- Thermal interface materials (TIM) between TFT glass and heatsink

**Convection**:
- Ventilation gaps in housing
- Natural convection for low-power displays
- Forced air for higher power (fans)

**Active Cooling**:

**Fans**:
- Axial or centrifugal fans
- Airflow: 10-50 CFM typical
- Noise consideration: <30 dBA target

**Liquid Cooling**:
- Used in ultra-bright cinema or outdoor displays
- Cold plate behind display
- Liquid circulation (water or glycol)
- Chiller system

**Peltier Coolers**:
- Thermoelectric cooling for precision temperature control
- High power consumption
- Used in specialized applications (test equipment)

**Thermal Vias**:

**Concept**: Vertical conductive paths through glass substrate

**Implementation**:
- Laser-drilled holes filled with conductive paste or plated metal
- Connect LED cathode/anode to back-plate heat sink
- Pitch: 100-500 μm

**Effectiveness**:
- Reduces junction temperature by 15-30°C
- Critical for small chip sizes (<30 μm) at high brightness

## Modular Design and Tiling

### Samsung The Wall Approach

**Cabinet Design**:
- 16:9 aspect ratio modules: 600 mm × 337.5 mm
- Pixel pitch: 0.84 mm (P0.84)
- Seamless edge design with black seal technology
- Magnetically attached to mounting frame

**Tiling Configuration**:
- 110": 2×1 cabinets
- 146": 2×2 cabinets
- 219": 4×2 cabinets
- 292": 4×3 cabinets

**Seam Minimization**:
- Precision machined cabinet edges (±10 μm)
- Calibrated pixel brightness near edges
- Bezel width: <0.5 mm effectively invisible at >2m viewing distance

**Advantages of Modular**:
- Any size and aspect ratio possible
- Failed module can be replaced
- Easier transport and installation
- Manufacturing yield per module

**Challenges**:
- Seam visibility (even if minimized)
- Color matching between modules
- Complexity of multi-module calibration
- Higher system cost

### Monolithic vs. Modular Trade-offs

**Monolithic** (single large panel):
- Seamless image
- Simpler system
- Limited sizes (constrained by manufacturing equipment)
- Higher manufacturing risk (single defect scraps entire panel)

**Modular**:
- Unlimited size
- Serviceable
- Seams present (even if minimal)
- More complex electronics and calibration
- Higher cost per area

**Market Segmentation**:
- Smartphones, tablets, laptops: Monolithic (≤15")
- TVs: Monolithic if possible, modular for ultra-large (>100")
- Commercial/cinema: Modular (flexibility and serviceability)

## Power Supply and Distribution

### Power Requirements

**Voltage Levels**:
- LED drive: 3-5V (depending on series configuration)
- TFT logic: 15V, -7V, 3.3V, 5V
- Driver ICs: 3.3V, 5V, 12V
- Total: Multiple voltage rails

**Current Requirements**:

For 65" 4K display:
- Peak power (full white): 300-500W
- Typical content (video): 100-200W
- Standby: 0.5-2W

**Power Supply Design**:
- AC-DC converter (85-265V AC to 12V or 24V DC)
- DC-DC converters for multiple rails
- Efficiency target: >90%
- Power factor correction (PFC) for compliance

### Power Distribution Challenges

**Voltage Drop**:
- Long traces can have significant resistance
- IR drop reduces LED brightness at far corners
- Mitigation: Multiple power feed points, wider traces, copper thickness increase

**EMI (Electromagnetic Interference)**:
- Switching power supplies generate EMI
- High-frequency PWM can radiate
- Shielding and filtering required for regulatory compliance (FCC, CE)

**Ground Loops**:
- Multiple ground paths can cause noise coupling
- Single-point grounding or star grounding topology
- Careful PCB layout

## Encapsulation and Protection

### Purpose of Encapsulation

**Protection from**:
- Moisture and oxygen (LED degradation)
- Mechanical damage
- Dust and particles
- UV radiation (for color conversion QDs)

### Encapsulation Materials

**Epoxy Resins**:
- Transparent, protective
- UV-curable or thermal-cure
- Good adhesion
- May yellow over time with UV exposure

**Silicone**:
- Excellent UV stability
- Flexible
- Lower modulus (less stress on LEDs)
- Higher cost than epoxy

**Glass Lids**:
- Hermetic seal possible
- Excellent barrier properties
- Used in OLED, applicable to MicroLED
- Frit bonding or adhesive bonding to TFT substrate

**Thin-Film Encapsulation (TFE)**:
- Alternating layers of organic and inorganic materials
- SiNx/polymer/SiNx stacks
- Thin and conformal
- Expensive

### Cover Glass and Touch Integration

**Cover Glass Functions**:
- Mechanical protection
- Optical surface quality
- Touch sensor substrate (if touch-enabled)
- Anti-reflective and anti-fingerprint coatings

**Touch Technologies**:

**Capacitive Touch**:
- Standard for smartphones and tablets
- ITO or metal mesh sensors
- Can be on-cell (integrated into display) or separate cover glass

**In-Display Fingerprint**:
- Optical or ultrasonic sensors
- Requires transparent display stack for optical
- Ultrasonic less dependent on transparency

**Integration**:
- Lamination with optical clear adhesive (OCA)
- Air gaps eliminated for best optical performance
- Alignment precision for on-cell touch

## System Calibration

### Factory Calibration Process

**1. Defect Mapping**:
- Identify and log all defective pixels
- Store defect map in TCON memory
- Enable defect masking or compensation

**2. Brightness Calibration**:
- Measure light output of each pixel
- Calculate correction factors
- Store in non-volatile memory (EEPROM, flash)
- Apply during operation to normalize brightness

**3. Color Calibration**:
- Measure color coordinates (x,y or u',v')
- Calculate RGB mixing ratios for target white point
- Store per-pixel or per-region corrections

**4. Gamma Calibration**:
- Characterize gray-scale response
- Fit to target gamma curve (2.2 or 2.4 typical)
- Generate lookup tables (LUTs)

**5. Uniformity Optimization**:
- Measure across display at multiple brightness levels
- Optimize for best trade-off between brightness and uniformity
- Typically accept 10-20% peak brightness reduction for <5% uniformity

### Calibration Data Storage

**Memory Requirements**:

For pixel-level calibration (4K display):
- 8.3 million pixels × 3 colors × 8 bits = 200 Mb
- Practical: Store coefficients, not full LUTs (reduce to 10-50 Mb)
- Alternative: Zone-based calibration (1,000-10,000 zones)

**Storage Location**:
- TCON on-board flash memory
- Separate EEPROM modules
- System-level storage (for smart displays)

### In-Field Calibration

**Brightness Adjustment**:
- Ambient light sensors adjust brightness
- User preference settings

**Aging Compensation**:
- Predict LED brightness loss over time
- Gradually increase drive current to maintain brightness
- More critical for OLED than MicroLED (less aging)

**Professional Displays**:
- Built-in colorimeters or integration with external meters
- Periodic re-calibration (monthly or quarterly)
- Critical for color-critical applications (film, broadcast)

## Conclusion

Display assembly and integration represents the convergence of multiple advanced technologies: TFT fabrication, semiconductor packaging, optical engineering, thermal management, and precision calibration. Success requires mastery of each domain and careful optimization of the integrated system.

Current state-of-the-art MicroLED displays (e.g., Samsung The Wall) demonstrate that all necessary integration technologies exist and work at commercial scale, albeit at premium price points. Cost reduction will come from process refinement, yield improvement, automation, and economies of scale as volume increases.

The WIA-SEMI-010 standard defines TFT specifications, driver interface protocols, thermal requirements, and calibration procedures to enable industry standardization and multi-vendor system integration.

---

**Key Integration Specifications:**
- TFT backplane: LTPS (small), Oxide (large), CMOS (micro-displays)
- Driver mounting: COG (compact), COF (thin bezel), COB (modular)
- Grayscale: 10-12 bit (PWM or hybrid PWM+AM)
- Thermal management: <85°C junction temperature target
- Calibration: <5% brightness uniformity, ΔE<2 color uniformity
- Modular tiling: <1 mm effective seam width

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
