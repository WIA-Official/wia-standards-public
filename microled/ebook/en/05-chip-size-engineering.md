# Chapter 5: Chip Size Engineering and Optimization

## The Chip Size Paradox

MicroLED chip size represents a fundamental trade-off in display design: smaller chips enable higher pixel density and resolution, but create exponentially greater manufacturing challenges. Understanding this trade-off is essential for optimizing MicroLED displays for specific applications.

## Chip Size Categories

### Ultra-Micro (<10 μm)

**Applications**: Ultra-high-resolution near-eye displays for AR/VR, microdisplays

**Advantages**:
- Enables pixel densities >3,000 PPI
- Eliminates screen door effect in near-eye applications
- Smallest possible display form factors
- Highest potential resolution per unit area

**Manufacturing Challenges**:
- Transfer yield <99% with current technologies
- Lithography requires advanced equipment (DUV or i-line steppers)
- Pick-and-place nearly impossible
- High defect rate from edge effects
- Current spreading limited by small lateral dimensions
- Requires <1 μm placement accuracy

**Electrical Considerations**:
- Operating current: 1-50 μA per chip
- Forward voltage: 2.5-3.2V
- Power per chip: 2.5-160 μW
- Heat dissipation less critical due to low power
- Contact resistance becomes significant portion of total resistance

**Optical Performance**:
- Light extraction efficiency reduced due to small dimensions
- Difficult to optimize photonic structures at <10 μm scale
- Color uniformity challenging due to edge effects
- Requires careful optical design to maintain efficiency

**Cost Drivers**:
- Chips per wafer: >10 million (8-inch wafer)
- Fabrication cost per chip: <$0.0001
- Transfer cost dominates: $0.001-0.01 per chip
- Overall cost: Transfer-limited, not chip fabrication limited

**Industry Status**: Research and early development, no commercial products yet

### Small (10-30 μm)

**Applications**: Smartphones, tablets, laptop displays, premium smartwatches

**Advantages**:
- High pixel density (>500 PPI for smartphones)
- Reasonable manufacturing yields (99-99.9% achievable)
- Good optical efficiency
- Suitable for mass market products

**Chip Count Examples**:
- 6.5" smartphone (OLED-class resolution): 13-15 million RGB LEDs
- 13" laptop (2560×1600): 12.3 million RGB LEDs
- Apple Watch (1.9"): 800,000 RGB LEDs

**Manufacturing**:
- Standard lithography adequate (i-line, 365 nm)
- Elastomer stamp transfer viable
- LIFT laser transfer practical
- Placement accuracy requirement: ±2 μm

**Electrical Design**:
- Operating current: 50-200 μA
- Power per chip: 125-640 μW
- Thermal management moderate concern
- TFT backplane complexity moderate
- Passive matrix not practical; active matrix required

**Optical Performance**:
- Light extraction efficiency: 40-60%
- Photonic structures (PSS, photonic crystals) effective at this scale
- Color uniformity good with proper binning
- Viewing angle: >170° (Lambertian emission)

**Cost Analysis**:
- Chips per 8-inch wafer: 1-4 million
- Fabrication cost: $0.0003-0.001 per chip
- Transfer cost: $0.001-0.005 per chip
- For 6.5" smartphone: $15-75 for LED chips and transfer alone
- Target competitive with OLED: Requires <$50 total LED cost

**Industry Status**: Active development by Apple (rumored), Samsung, CSOT; commercial products expected 2026-2028

### Medium (30-75 μm)

**Applications**: Wearables, automotive displays, AR glasses projection systems

**Advantages**:
- Excellent manufacturing yields (>99.9%)
- Mature transfer processes
- Optimal efficiency for given size
- Good balance of performance and manufacturability

**Applications Detail**:

**Smartwatches (Medium-Resolution)**:
- 1.5-2" diagonal, 300-400 PPI
- Chip size: 40-60 μm
- Total chips: 500,000-1.5 million
- Market: Mainstream smartwatches

**Automotive HUDs**:
- Small image generators with optical magnification
- Chip size: 50-75 μm
- Total chips: 500,000-2 million
- Brightness requirement: >10,000 nits
- Temperature range: -40°C to +105°C

**AR Microdisplay Imagers**:
- 0.3-0.7" diagonal
- Chip size: 30-50 μm
- Magnified optically for near-eye viewing
- Total chips: 1-5 million RGB

**Manufacturing**:
- Conventional photolithography
- All transfer methods viable
- Very high yield achievable (>99.95%)
- Inspection and repair straightforward

**Electrical Characteristics**:
- Current: 200-1000 μA
- Power: 0.5-3.2 mW
- Thermal management important for high-brightness applications
- Can use simpler driver schemes in some applications

**Optical Design**:
- Excellent light extraction with PSS or surface texturing
- Efficiency: 50-70% extraction
- Uniform emission patterns
- Photonic enhancement effective

**Cost**:
- Fabrication: $0.0005-0.002 per chip
- Transfer: $0.0005-0.002 per chip
- Very cost-competitive for wearables and automotive
- High-margin applications justify current costs

**Industry Status**: Commercially viable today; used in premium wearables and automotive prototypes

### Large (75-150 μm)

**Applications**: Transparent displays, large-format signage, specialized industrial displays

**Advantages**:
- Easiest manufacturing
- Highest yields (>99.99%)
- Can use pick-and-place effectively
- Excellent thermal management due to larger chip size
- Highest brightness per chip possible

**Transparency Applications**:
- Chip size: 100-150 μm, widely spaced
- Fill factor: 5-15% (85-95% transparent)
- Applications: Automotive windows, retail displays, architectural glass
- Brightness: 1,000-3,000 nits despite low fill factor

**Large Format Displays**:
- Pixel pitch: 0.5-2.0 mm
- Viewing distance: 2-10 meters
- Applications: Stadium screens, command centers, rental/staging

**Manufacturing**:
- Simple pick-and-place adequate
- High-speed assembly possible
- Manual rework feasible
- Inspection straightforward

**Electrical**:
- Current: 1-10 mA per chip
- Power: 2.5-32 mW
- Heat dissipation critical consideration
- Requires thermal vias or heat sinks
- Passive matrix possible for some applications

**Cost**:
- Chip cost $0.001-0.005
- Transfer cost $0.0005-0.002
- Total chip cost minor portion of display cost
- Cost dominated by TFT backplane, drivers, assembly

**Industry Status**: Mature technology, commercially deployed

## Physical Phenomena at Different Scales

### Current Density and Efficiency Droop

**Efficiency Droop**: LED efficiency decreases at high current density

**Physical Mechanisms**:
- Auger recombination (non-radiative)
- Carrier overflow from quantum wells
- Junction heating

**Size Dependence**:

For same brightness, smaller chips operate at higher current density:

- 10 μm chip @ 50 μA: 637 A/cm² (assuming 2.5 μm emission area)
- 50 μm chip @ 1 mA: 509 A/cm² (assuming 20 μm emission area)

Smaller chips may actually benefit from lower current per chip despite higher current density in some designs.

**Optimization Strategy**:
- Design emission area to optimize current density
- Use multiple quantum wells for current distribution
- Engineer tunnel junction designs for improved injection

### Current Spreading

**Challenge**: Uniform current distribution across chip active area

**Small Chips (<20 μm)**:
- Lateral resistance significant
- Current crowding near contact
- Transparent conducting oxide (TCO) layer critical
- ITO thickness: 100-200 nm optimized for transparency and conductivity

**Large Chips (>75 μm)**:
- Current spreading excellent with proper TCO
- Can tolerate higher contact resistance
- Less sensitive to TCO thickness uniformity

**Current Spreading Length**:
LSP = √(ρTFT × tTCO / ρTCO)

Where:
- ρTFT: Resistivity of TCO
- tTCO: TCO thickness
- ρTCO: Sheet resistance

For effective spreading, chip dimension should be ≤2-3× LSP

### Light Extraction Efficiency

**Fundamental Challenge**: High refractive index of GaN (n ≈ 2.5) traps light

**Critical Angle**: θc = arcsin(1/n) ≈ 23.6° for GaN/air interface

**Extraction Efficiency Without Enhancement**:
- Theoretical: ~4% (only light within escape cone)
- Practical with surface roughening: 20-30%

**Enhancement Techniques**:

**Patterned Sapphire Substrate (PSS)**:
- Effective for all chip sizes
- Typical enhancement: 2-3× vs. planar
- Pattern pitch: 2-4 μm (comparable to wavelength)

**Photonic Crystals**:
- Most effective for medium chips (20-75 μm)
- Pattern pitch: 200-400 nm
- Enhancement: 1.5-2.5×
- Difficult to implement uniformly on very small chips

**Surface Texturing**:
- Random or ordered roughening of p-GaN surface
- Enhancement: 1.5-2×
- Scalable to all chip sizes

**Chip Shape**:
- Truncated inverted pyramid (TIP) geometry
- Mesa sidewalls angled to redirect trapped light
- More effective for larger chips

**Size Optimization**:
- 10 μm: Limited enhancement possible, ~30-40% extraction
- 30 μm: Good enhancement with PSS+texturing, 40-60% extraction
- 50 μm: Excellent enhancement with photonic structures, 50-70% extraction
- >100 μm: Maximum extraction, 60-75% achievable

### Thermal Management

**Heat Generation**: P = I × Vf × (1 - ηEQE)

Where:
- I: Drive current
- Vf: Forward voltage
- ηEQE: External quantum efficiency

**Thermal Resistance**: θJA = (Tj - Ta) / P

**Small Chips**:
- Low power per chip (0.1-1 mW)
- Heat spreads easily to substrate
- Junction temperature rise: <10°C typically
- Thermal management not critical

**Medium Chips**:
- Moderate power (1-5 mW)
- Temperature rise: 10-30°C
- May require thermal vias in substrate
- Important for high-brightness applications

**Large Chips**:
- High power (5-50 mW)
- Temperature rise: 30-80°C without thermal management
- Requires thermal vias, heat sinks, or active cooling
- Junction temperature directly impacts efficiency and lifetime

**Thermal Design Strategies**:
- Sapphire substrate removal (reduces thermal resistance)
- Copper thermal vias through TFT backplane
- Backside metal heat spreaders
- For large high-power displays: Liquid cooling

## Pixel Architecture Design

### Subpixel Configuration

**Standard RGB Stripe**:
- Three rectangular subpixels (R, G, B)
- Chip size equal for all colors
- Simple driver addressing
- Used in most displays

**PenTile or Diamond Pixel**:
- Shared subpixels between adjacent pixels
- Can use different chip sizes for R, G, B
- Reduces total chip count by 33%
- More complex rendering
- Used in OLED smartphones, possible for MicroLED

**Quad Pixel (RGBW)**:
- Adds white subpixel
- Increases brightness and efficiency
- Requires 33% more chips
- Driver complexity increases

### Chip Size Optimization by Color

**Different Sizes by Color**:

Blue LEDs (450 nm):
- Highest efficiency
- Can use smaller chips for same brightness
- Example: 20 μm blue chips

Green LEDs (520 nm):
- Moderate efficiency
- Medium-size chips
- Example: 25 μm green chips

Red LEDs (620 nm):
- Lowest efficiency (especially for AlGaInP)
- Larger chips needed for brightness matching
- Example: 30 μm red chips (or color conversion)

**Advantages**:
- Optimizes efficiency vs. chip count
- Reduces total LED cost
- Can improve uniformity

**Challenges**:
- More complex transfer (different chip sizes)
- Harder to source/bin multiple sizes
- Increased design complexity

### Chip Spacing and Fill Factor

**Fill Factor**: Ratio of LED chip area to total pixel area

**High Fill Factor (>50%)**:
- Brighter display possible
- Less precision required for TFT backplane
- Higher capacitance (slower response)
- More visible structure in transmission (transparent displays)

**Low Fill Factor (<30%)**:
- More transparent (for transparent displays)
- Better heat dissipation
- Tighter placement tolerance required
- Lower maximum brightness

**Typical Values**:
- Smartphones: 30-40% fill factor
- TVs: 25-35% fill factor
- Transparent displays: 5-15% fill factor
- Smartwatches: 35-45% fill factor

## Application-Specific Optimization

### Smartphones (Optimal: 15-25 μm)

**Requirements**:
- Resolution: 400-500 PPI
- Brightness: 800-1200 nits (HDR peaks to 1500+)
- Power efficiency: <4W full white
- Thickness: <0.5 mm display module

**Optimization**:
- 15-25 μm chips balance yield and performance
- Fill factor: 30-35%
- Color conversion layer possible to use single blue LED type

**Challenges**:
- Cost target: <$50 for display (compete with OLED)
- Requires >99.99% transfer yield
- Touch integration

### AR Glasses (Optimal: 3-8 μm)

**Requirements**:
- Resolution: 2000-4000 PPI
- Brightness: 5000-10000 nits (for daylight AR)
- Display size: 0.3-0.7 inch diagonal
- Weight: <2 grams per display

**Optimization**:
- 3-8 μm chips necessary for PPI requirement
- Extreme efficiency critical for battery life
- Color conversion likely required (blue + QD)

**Challenges**:
- Transfer yield <<99.9% currently
- Optical system integration
- Heat dissipation in enclosed form factor

### Large Format (Optimal: 100-200 μm)

**Requirements**:
- Pixel pitch: 0.6-2.0 mm
- Brightness: 600-2000 nits
- Viewing distance: 2-10 meters
- Modular/tileable design

**Optimization**:
- 100-200 μm chips provide easy manufacturing
- Pick-and-place transfer adequate
- Excellent brightness per chip possible

**Challenges**:
- Requires millions of chips even for moderate resolution
- Seamless tiling (The Wall addresses with precision cabinets)
- Power consumption (kilowatts for large displays)

## Future Trends in Chip Size

### Vertical GaN LEDs

**Concept**: Remove sapphire substrate, create vertical current path

**Advantages**:
- Better current spreading
- Improved thermal conductivity
- Allows smaller chips with uniform emission
- Can enable <10 μm chips with good performance

**Status**: Commercial for standard LEDs, development for MicroLED

### Nanowire and Nanorod LEDs

**Structure**: Arrays of vertical GaN nanowires (diameter <500 nm)

**Advantages**:
- Relaxed lattice strain (enables red InGaN on GaN)
- High light extraction (waveguide effects)
- Potential for single-chip RGB

**Challenges**:
- Manufacturing complexity
- Electrical contact to nanowire arrays
- Yield and uniformity

**Status**: University research, 5-10 years from commercialization

### Wafer-Level Displays

**Concept**: Build entire display on single wafer, no transfer needed

**Advantages**:
- Eliminates transfer challenges
- Highest precision and uniformity
- Lowest cost for small displays

**Limitations**:
- Display size limited to wafer size
- Not suitable for large displays
- TFT integration on GaN substrate challenging

**Target Applications**: Smartwatches, AR microdisplays

**Status**: Research phase, promising for micro-displays

## Conclusion

Chip size selection represents the most fundamental design choice in MicroLED display development. The optimal size depends on application requirements, balancing:

- Pixel density and resolution needs
- Manufacturing yield and cost
- Optical efficiency and brightness
- Thermal management
- Transfer technology capabilities

Current state-of-the-art:
- <10 μm: Research phase
- 10-30 μm: Active development for smartphones/tablets
- 30-75 μm: Production-ready for wearables and automotive
- 75-200 μm: Mature technology for large format

The WIA-SEMI-010 standard provides chip size specifications, performance metrics, and testing protocols for each size category, enabling consistent industry communication and benchmarking.

---

**Chip Size Guidelines by Application:**
- AR/VR microdisplays: 3-10 μm
- Smartphones/tablets: 15-30 μm
- Smartwatches: 30-60 μm
- Automotive: 40-75 μm
- TVs (future): 20-40 μm
- Large format/signage: 75-200 μm

---

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity
