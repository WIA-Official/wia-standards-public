# Chapter 2: Fundamentals of Flexible Displays

## Understanding the Science Behind Bendable Screens

---

### 2.1 Introduction to Flexible Display Technology

Flexible displays represent one of the most significant paradigm shifts in electronic display technology since the transition from CRT to flat panels. Unlike traditional rigid displays built on glass substrates, flexible displays utilize polymer or ultra-thin glass materials that can bend, fold, or even stretch while maintaining full functionality.

#### What Makes a Display "Flexible"?

A display is considered flexible when it meets these criteria:

1. **Mechanical Flexibility**: Ability to bend to a radius of curvature < 10mm without mechanical failure
2. **Optical Integrity**: Maintains image quality (brightness, color accuracy, uniformity) during and after flexing
3. **Electronic Functionality**: All pixels, drivers, and circuits continue operating through flex cycles
4. **Structural Durability**: Withstands repeated flexing (typically 100,000+ cycles) without degradation
5. **Touch Sensitivity Preservation**: Maintains capacitive or resistive touch response across the flexible area

#### Classification of Flexible Displays

The WIA-SEMI-011 standard classifies flexible displays into three primary categories:

**Type 1: Foldable Displays**
- Bend radius: 1.5mm to 5mm
- Flex cycles: 200,000+ minimum
- Applications: Smartphones, tablets, laptops
- Examples: Samsung Galaxy Z Fold, Huawei Mate X

**Type 2: Rollable Displays**
- Bend radius: 2mm to 8mm (continuous curvature)
- Extension ratio: 2:1 to 4:1 (compact to expanded)
- Applications: Portable monitors, expandable phones, rollable TVs
- Examples: LG Rollable, TCL Fold 'n Roll

**Type 3: Stretchable Displays**
- Elastic deformation: 20% to 50% biaxial stretching
- Recovery: >95% return to original dimensions
- Applications: Wearables, conformable automotive displays, medical devices
- Examples: Research prototypes from KAIST, Stanford

### 2.2 Materials Science Fundamentals

#### Substrate Materials

The substrate is the foundation of any flexible display, providing mechanical support while allowing flexing.

**Ultra-Thin Glass (UTG)**

Properties:
- Thickness: 25-50 μm (thinner than human hair)
- Composition: Alkali-free aluminosilicate glass
- Young's Modulus: 70-80 GPa
- Minimum Bend Radius: 1.5-3mm
- Surface Hardness: 6-7 Mohs (scratch resistant)
- Thermal Stability: Can withstand 400°C+ processing temperatures

Advantages:
- Superior scratch resistance vs. polymers
- Excellent optical clarity and low haze
- Dimensional stability (low thermal expansion)
- Gas barrier properties (prevents water/oxygen ingress)
- Smooth surface finish (<1nm roughness)

Disadvantages:
- Higher cost ($15-25 per unit vs. $2-5 for polymer)
- Risk of catastrophic failure if bent beyond critical radius
- Requires specialized handling equipment
- Limited stretchability

Manufacturing Process:
1. Float glass process creates continuous ribbon
2. Chemical strengthening via ion exchange (K+ replaces Na+)
3. Precision thinning to 25-50 μm
4. Edge polishing to prevent micro-crack propagation
5. Protective coating application

**Polyimide (PI) Films**

Properties:
- Thickness: 20-100 μm
- Glass Transition Temperature: 250-400°C
- Young's Modulus: 2.5-8 GPa
- Coefficient of Thermal Expansion: 3-40 ppm/°C
- Transparency: >85% @ 550nm (colorless PI)

Advantages:
- Highly flexible (can bend to <1mm radius)
- Lightweight (density ~1.4 g/cm³)
- Excellent thermal stability
- Chemical resistance
- Low cost and scalable manufacturing

Disadvantages:
- Lower scratch resistance (2-3 Mohs)
- Yellowish tint in standard formulations
- Higher gas permeability requires additional barrier layers
- UV degradation over time

Types of Polyimide:
- **Standard PI**: Brown/amber color, lowest cost, used in flex circuits
- **Transparent PI (CPI)**: Colorless, higher cost, optical-grade for displays
- **Black PI**: For light shielding and border areas
- **Patternable PI**: Photo-definable for direct patterning

**Polyethylene Terephthalate (PET)**

Properties:
- Thickness: 50-200 μm
- Maximum Processing Temperature: 120-150°C
- Young's Modulus: 2-4 GPa
- Cost: $1-3 per m²

Advantages:
- Lowest cost substrate option
- High transparency (>90%)
- Good dimensional stability
- Well-established manufacturing

Disadvantages:
- Low thermal stability (cannot withstand OLED processing >150°C)
- Requires low-temperature backplane processes
- Higher gas permeability
- Limited to less demanding applications

**Emerging Substrate Materials**

Research and development efforts focus on:

- **Thin Flexible Silicon**: 5-10 μm crystalline silicon on polymer, combines high performance with flexibility
- **2D Materials**: Graphene and MoS₂ for ultimate thinness and flexibility
- **Self-Healing Polymers**: Autonomously repair micro-cracks from repeated flexing
- **Shape-Memory Alloys**: Can return to predetermined shapes after deformation

#### Barrier Layers

Organic materials in OLED displays are highly sensitive to water vapor and oxygen, requiring effective encapsulation.

**Requirements for Flexible Barriers:**
- Water Vapor Transmission Rate (WVTR): <10⁻⁶ g/m²/day
- Oxygen Transmission Rate (OTR): <10⁻⁵ cm³/m²/day
- Flexibility: Must bend with substrate without cracking
- Optical Properties: High transparency (>90%), low haze (<2%)
- Thickness: <10 μm to maintain flexibility

**Multi-Layer Barrier Structures:**

Typical structure: [Inorganic/Organic/Inorganic/Organic/Inorganic]

Inorganic Layers (50-200 nm each):
- Silicon Nitride (SiNₓ): Excellent barrier, but brittle
- Silicon Oxide (SiOₓ): Lower barrier but more flexible
- Aluminum Oxide (Al₂O₃): Best barrier properties via ALD

Organic Layers (1-5 μm each):
- Polyacrylates: Planarization and stress relief
- Epoxy resins: Fill pinholes in inorganic layers
- Hybrid organic-inorganic: Combines benefits of both

Deposition Methods:
- **PECVD** (Plasma-Enhanced Chemical Vapor Deposition): High throughput, moderate flexibility
- **ALD** (Atomic Layer Deposition): Excellent conformality, pinhole-free, but slow
- **Sputtering**: Fast, good adhesion, but can introduce defects
- **Solution Processing**: Low cost, roll-to-roll compatible

**Advanced Barrier Technologies:**

- **Nano-Laminate Structures**: 100+ alternating layers, each <10nm thick
- **Graphene Barriers**: Single-atom-thick impermeable layer
- **Graded Composition Barriers**: Gradually varying composition to minimize stress
- **Atomic Layer Etching (ALE)**: Creates ultra-smooth interfaces for better sealing

### 2.3 Display Technology Types

#### Active Matrix OLED (AMOLED)

AMOLED is the dominant technology for flexible displays due to its emissive nature (no backlight needed) and compatibility with flexible substrates.

**Structure (bottom to top):**

1. **Flexible Substrate**: PI or UTG, 25-100 μm
2. **Barrier Layer**: Multi-layer inorganic/organic, 1-5 μm
3. **Backplane**: TFT array for pixel switching
   - Oxide TFT: IGZO (Indium Gallium Zinc Oxide), higher mobility, better stability
   - LTPS (Low-Temperature Polysilicon): Higher performance but more complex
   - Organic TFT: Ultimate flexibility, but lower performance
4. **Planarization Layer**: Smooths surface, 1-2 μm
5. **OLED Stack**:
   - Anode (ITO or silver): 100-200 nm, transparent
   - Hole Injection Layer (HIL): 5-10 nm
   - Hole Transport Layer (HTL): 20-50 nm
   - Emissive Layer (EML): 20-40 nm (RGB or white + color filters)
   - Electron Transport Layer (ETL): 20-50 nm
   - Cathode (Al or Ag): 100 nm, reflective
6. **Encapsulation**: Multi-layer barrier, 1-10 μm
7. **Touch Sensor**: Capacitive matrix, optional

**Pixel Architectures:**

RGB OLED:
- Separate red, green, blue sub-pixels
- Fabricated via fine metal mask (FMM) evaporation
- Best color purity and efficiency
- Resolution limited by mask precision (~500 PPI practical limit)

White OLED + Color Filter (WOLED-CF):
- Single white emissive layer
- Color filters create RGB
- Easier manufacturing, higher yield
- Lower light efficiency (filters absorb 60-70% of light)
- Used in LG OLED TVs

Tandem OLED:
- Stacked emissive units connected via charge generation layer
- 2-3x lifetime at same brightness
- Higher efficiency
- More complex structure

QD-OLED (Quantum Dot OLED):
- Blue OLED backlight
- Quantum dots convert blue to red and green
- Best color gamut (>90% DCI-P3)
- Combines OLED contrast with QD color

**Driving Schemes:**

2T1C (2 Transistors, 1 Capacitor):
- Most common pixel circuit
- Transistor 1: Switching (select pixel)
- Transistor 2: Driving (controls OLED current)
- Capacitor: Stores voltage between refresh cycles

7T1C:
- Compensation for threshold voltage shifts
- Used in high-reliability applications
- Reduces mura (non-uniformity)

External Compensation:
- Measures TFT characteristics via sensing circuits
- Adjusts drive voltage to compensate for aging
- Used in premium displays (Samsung flagship phones)

#### Micro-LED on Flexible Substrates

Micro-LED technology combines the benefits of LED (long lifetime, high efficiency) with OLED's emissive nature.

**Advantages over OLED:**
- Brightness: 10,000+ nits (vs. 1,000 nits for OLED)
- Lifetime: 100,000+ hours (vs. 30,000 hours for OLED)
- Power Efficiency: 2-3x better at high brightness
- No burn-in or image retention
- Wide color gamut without color filters

**Challenges for Flexible Micro-LED:**
- **Mass Transfer**: Moving millions of 5-50 μm LEDs from growth substrate to display
  - Pick-and-place: Precise but slow (<100 chips/second)
  - Laser-assisted transfer: Faster (10,000+ chips/second)
  - Fluid assembly: Self-alignment via surface tension
  - Roll-to-roll printing: Ultimate scalability

- **Interconnection**: Electrical connection of micro-LEDs to flexible backplane
  - Stretchable interconnects: Serpentine or mesh patterns
  - Anisotropic conductive adhesives (ACA): Pressure-activated bonding
  - Laser soldering: High precision, thermal management challenge

- **Substrate Compatibility**: GaN LEDs typically grown on sapphire at 1000°C+
  - Epitaxial lift-off (ELO): Separate LED layer from growth substrate
  - Low-temperature bonding: Transfer to flexible substrate at <200°C
  - Thinning: Reduce LED thickness to <5 μm for flexibility

**Current Status (2025):**
- Large displays (>100 inch): Commercial products (Samsung The Wall)
- Medium displays (10-100 inch): Pre-production (LG, Sony)
- Small displays (<10 inch, flexible): Research prototypes (KAIST, MIT)

Expected timeline for flexible micro-LED smartphones: 2028-2030

### 2.4 Physics of Bending

Understanding the mechanical behavior of materials under bending stress is crucial for designing durable flexible displays.

#### Neutral Plane Concept

When a multilayer structure bends, the outer layers experience tensile stress (stretching) while inner layers experience compressive stress. At the **neutral plane**, stress is zero.

**Neutral Plane Position:**

For a symmetric structure: z_neutral = thickness / 2

For multilayer with different Young's moduli:
```
z_neutral = Σ(E_i × A_i × z_i) / Σ(E_i × A_i)
```
Where:
- E_i = Young's modulus of layer i
- A_i = Cross-sectional area of layer i
- z_i = Distance of layer i from reference

**Design Strategy:**
Place critical components (active OLED materials, TFT channels) near the neutral plane to minimize stress.

#### Strain Calculation

Strain (ε) = (Distance from neutral plane) / (Bend radius)

For a 50 μm thick substrate bent to 3mm radius:
- Maximum strain at surface = 25 μm / 3000 μm = 0.833%

Critical strain limits:
- Brittle ceramics (ITO, SiN): <0.5% before cracking
- Polymers: 1-5% elastic, >10% plastic deformation
- Metals: 0.2-2% elastic, >5% plastic deformation
- OLED organics: <1% before delamination or cracking

**Fracture Mechanics:**

Griffith Criterion for crack propagation:
```
σ_c = √(2 × E × γ / (π × a))
```
Where:
- σ_c = Critical stress for crack propagation
- E = Young's modulus
- γ = Surface energy
- a = Crack length

**Key Insight:** Preventing initial crack formation is more important than crack arrest, because micro-cracks can propagate catastrophically in brittle materials.

#### Fatigue and Creep

**Fatigue:** Degradation from repeated stress cycling

S-N Curve (Stress vs. Number of cycles):
- High stress → Fewer cycles to failure
- Low stress → Millions of cycles possible

Endurance Limit: Stress level below which infinite cycles are possible (for some materials like steel; polymers typically don't have true endurance limit)

For flexible displays:
- Target: >200,000 cycles at operational bend radius
- Safety factor: Design for <50% of ultimate stress
- Stress concentrations: Avoid sharp corners, use gradual transitions

**Creep:** Time-dependent deformation under constant stress

Polymer substrates exhibit creep at room temperature:
- 1% stress → 0.1-0.5% creep strain over 5 years
- Higher temperatures accelerate creep (factor of 2-10 per 10°C)

Mitigation strategies:
- Use high-Tg polymers (PI over PET)
- Minimize constant stress (open fold angle when not in use)
- Rigid frames to constrain edges

### 2.5 Optical Considerations

#### Refractive Index Matching

Minimize reflections at interfaces by matching refractive indices:

Reflection at interface = [(n₁ - n₂) / (n₁ + n₂)]²

Typical values:
- Air: n = 1.0
- Polymer: n = 1.5-1.7
- Glass: n = 1.5
- ITO: n = 1.8-2.0

**Anti-Reflection Coatings:**
- Quarter-wave layers: thickness = λ/(4×n)
- Multi-layer coatings: <0.5% reflection across visible spectrum

#### Light Extraction Efficiency

OLED emits light isotropically, but substrate trapping limits extraction:

Internal extraction efficiency = 1 / (2n²)

For n=1.5 substrate: Only ~20% of light escapes initially

Enhancement techniques:
- Microlens arrays: Redirect trapped light, +30-50% brightness
- Low-index grids: Create scattering centers, +20-40%
- Nano-imprinting: Surface texturing, +40-60%
- External scattering layers: +10-20%, simple to implement

#### Color Shift with Viewing Angle

Cavity interference in OLED stack causes color shift:
- Blue shifts to cyan at high angles
- Red remains relatively stable

Mitigation:
- Broader cavity design (reduces intensity but improves angle)
- Micro-cavity arrays with varying thickness
- External diffusers (reduces peak brightness)

### 2.6 Thermal Management

Flexible displays face unique thermal challenges:

#### Heat Generation

Sources:
- TFT backplane: 0.1-0.5 W/cm² during operation
- OLED emission: 0.5-2 W/cm² at full brightness
- Driving ICs: 1-5 W total

#### Heat Dissipation

Challenges:
- Polymer substrates have low thermal conductivity (0.1-0.3 W/m·K vs. 1 W/m·K for glass)
- Cannot use traditional heat sinks in folding region
- Encapsulation layers add thermal resistance

Solutions:
- **Graphene heat spreaders**: 2,000+ W/m·K, <1 μm thick, flexible
- **Copper mesh**: 400 W/m·K, stretchable serpentine design
- **Phase-change materials**: Absorb heat during peaks, release gradually
- **Active cooling**: Micro-fans or liquid cooling for AR/VR headsets (not phones)

#### Temperature Effects on Performance

OLED characteristics vs. temperature:
- Brightness: Decreases ~0.5%/°C at constant current
- Color: Blue shifts with higher temperature
- Lifetime: Accelerated degradation at >60°C (exponential relationship)

Maximum safe operating temperatures:
- Consumer devices: 45°C display surface
- Industrial: 60°C
- Automotive: 85°C (requires special OLED materials)

### 2.7 Touch Integration

Flexible displays require flexible touch sensors:

#### On-Cell Touch

Touch sensor integrated into display structure:
- Advantages: Thinner, better optical performance, single lamination
- Challenges: More complex display fabrication, lower touch sensitivity

Implementation:
- Use metal mesh or ITO on TFE as touch electrodes
- Mutual capacitance sensing
- Noise shielding from display signals

#### Add-On Touch

Separate touch sensor laminated on top:
- Advantages: Simpler display fabrication, standard touch technology
- Challenges: Thicker, optical losses, two lamination steps

Touch Technologies for Flexible:
- **Metal Mesh**: Excellent flexibility, but visible under some conditions
- **Silver Nanowire**: Good flexibility and transparency, durability concerns
- **Graphene**: Ultimate flexibility, developing technology
- **Conductive Polymer**: PEDOT:PSS, moderate performance

#### Force Touch / Pressure Sensing

Implementation in flexible displays:
- Strain gauges in hinge area
- Capacitive pressure sensors between layers
- Piezoelectric films

Applications:
- Pressure-sensitive drawing
- 3D Touch gestures
- Palm rejection

---

### 2.8 Summary

This chapter covered the fundamental science and engineering principles behind flexible displays:

✅ **Materials**: UTG, PI, PET substrates and their properties
✅ **Barrier Technology**: Multi-layer encapsulation for OLED protection
✅ **Display Technologies**: AMOLED and emerging micro-LED
✅ **Mechanical Behavior**: Neutral plane, strain, fatigue, creep
✅ **Optical Design**: Light extraction, color stability, anti-reflection
✅ **Thermal Management**: Heat dissipation in flexible structures
✅ **Touch Integration**: On-cell and add-on flexible touch sensors

In **Chapter 3**, we'll explore foldable display technology in depth, including hinge mechanisms, crease management, and the engineering behind devices like the Samsung Galaxy Z Fold and Huawei Mate X.

---

**WIA-SEMI-011 | Chapter 2 | Fundamentals of Flexible Displays**

© 2025 SmileStory Inc. / WIA | 弘益人間 (Benefit All Humanity)
