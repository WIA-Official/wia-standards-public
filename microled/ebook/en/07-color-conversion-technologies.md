# Chapter 7: Color Conversion Technologies

## The Color Challenge in MicroLED

MicroLED displays face a fundamental color production challenge: while blue and green GaN-based LEDs achieve excellent efficiency (>60% external quantum efficiency for blue, >40% for green), red LEDs lag significantly behind. AlGaInP red LEDs, the traditional technology, suffer from poor efficiency at micro-scale (<20% EQE for chips <50 μm), and InGaN red LEDs, while improving, still underperform compared to blue and green.

Color conversion technologies offer an elegant solution: use highly efficient blue (or UV) MicroLEDs as excitation sources, and convert blue photons to red and green using quantum dots, phosphors, or other down-conversion materials. This approach potentially simplifies manufacturing (one LED type instead of three), improves red efficiency, and reduces cost—but introduces new challenges in conversion efficiency, color purity, and material stability.

## Fundamentals of Color Conversion

### Down-Conversion Principle

**Photoluminescence Process**:
1. High-energy photon (blue or UV) absorbed by conversion material
2. Electron promoted to excited state
3. Electron relaxes, emitting lower-energy photon (green or red)
4. Stokes shift: Energy difference dissipated as heat

**Energy Consideration**:
- Blue photon: ~2.7 eV (460 nm)
- Green photon: ~2.4 eV (520 nm)
- Red photon: ~2.0 eV (620 nm)

**Theoretical Conversion Efficiency**:
- Blue → Green: 2.4/2.7 = 89% (plus material quantum yield)
- Blue → Red: 2.0/2.7 = 74% (plus material quantum yield)

**Practical Efficiency**:
- Depends on material quantum yield (QY)
- Best QDs: >90% QY
- Best phosphors: 80-95% QY
- Total efficiency: Energy ratio × QY = 65-85% (green), 60-75% (red)

### Advantages of Color Conversion

**Manufacturing Simplicity**:
- Single LED type eliminates RGB alignment challenges
- Simpler mass transfer (uniform chip size)
- One epitaxial process instead of three
- Reduced binning complexity

**Red Efficiency Improvement**:
- QD-converted red can exceed native AlGaInP red efficiency
- Especially significant for micro-scale chips (<30 μm)
- Enables smaller red pixels without brightness penalty

**Color Gamut Extension**:
- Narrow-band QD emission (<30 nm FWHM)
- Wider color gamut than LED primaries
- Potential for >95% Rec. 2020 coverage

**Cost Reduction** (potential):
- Single LED wafer type reduces inventory and complexity
- May offset QD material costs at scale

### Challenges of Color Conversion

**Conversion Losses**:
- Stokes shift represents 10-40% energy loss
- Not all photons converted (scattering, transmission losses)
- Total system efficiency may be lower than native RGB

**Color Crosstalk**:
- Blue light leakage into red/green subpixels
- Reduces color purity and contrast
- Requires barrier structures

**Material Stability**:
- QDs and phosphors can degrade with heat, moisture, UV exposure
- Lifetime concerns for display applications
- Encapsulation critical

**Thickness and Optical Design**:
- Conversion layer must be thick enough for complete absorption
- Thick layers increase optical crosstalk between subpixels
- Requires careful engineering

**Material Cost**:
- High-performance QDs currently expensive
- Must achieve cost parity with native RGB LED approach

## Conversion Material Technologies

### Quantum Dots (QDs)

Quantum dots are nanoscale semiconductor crystals (2-10 nm diameter) that emit specific colors based on size due to quantum confinement effects.

**Material Systems**:

**Cd-Based QDs** (Cadmium Selenide/Sulfide):
- CdSe/ZnS core/shell structures
- Excellent optical properties:
  - Quantum yield: >90%
  - FWHM: 25-35 nm
  - Color tunability: 450-650 nm
- **Critical Issue**: Cadmium toxicity
- **Regulatory Status**: Restricted in EU (RoHS), limited exceptions for displays
- **Use**: Declining due to regulations

**Cd-Free QDs** (Indium Phosphide):
- InP/ZnS or InP/ZnSe/ZnS core/shell/shell
- Properties:
  - Quantum yield: 70-90% (improving)
  - FWHM: 35-45 nm (slightly broader than CdSe)
  - Color range: 500-650 nm
- **Advantages**: Non-toxic, regulatory compliant
- **Challenges**: Lower efficiency than Cd-QDs, manufacturing complexity
- **Status**: Leading technology for future QD displays

**Perovskite QDs**:
- CsPbX₃ (X = Cl, Br, I)
- Properties:
  - Quantum yield: >90%
  - FWHM: 15-30 nm (narrower than other QDs)
  - Color tunability: 410-700 nm
- **Advantages**: Exceptional color purity, solution processing
- **Challenges**: Stability (moisture, oxygen sensitivity), lead content
- **Status**: Active research, commercial viability uncertain

**Carbon QDs**:
- Carbon-based nanodots
- Properties:
  - Quantum yield: 30-70%
  - Broader emission spectrum
  - Low toxicity
- **Advantages**: Earth-abundant, non-toxic
- **Challenges**: Lower efficiency, less color control
- **Status**: Early research

**Quantum Dot Sizing**:
- Smaller QDs → blue-shifted emission (green)
- Larger QDs → red-shifted emission (red)
- Typical sizes: 3-6 nm (green), 6-10 nm (red)

**Synthesis**:
- Hot injection method (batch production)
- Continuous flow reactors (scale-up)
- Atomic layer deposition (for some applications)

**Encapsulation**:
- QDs are sensitive to oxygen, moisture
- Protective shells: ZnS, metal oxides
- Polymer or glass encapsulation in display

### Phosphors

Phosphors are crystalline materials doped with rare earth or transition metal ions that luminesce when excited.

**Material Systems**:

**Garnet Phosphors** (Green/Yellow):
- YAG:Ce (Yttrium Aluminum Garnet doped with Cerium)
- Properties:
  - Quantum yield: 85-95%
  - Peak emission: 530-560 nm
  - FWHM: 100-120 nm (broad)
- **Advantages**: Excellent stability, proven in LED lighting
- **Challenges**: Broad spectrum reduces color purity

**(Ba,Sr)₂SiO₄:Eu²⁺** (Green):
- Orthosilicate phosphor
- Properties:
  - Quantum yield: 80-90%
  - Peak: 505-520 nm
  - FWHM: 50-70 nm
- **Advantages**: Narrower than garnet, good stability
- **Applications**: LED backlights, potential for MicroLED

**Nitride Phosphors** (Red):
- (Ca,Sr)AlSiN₃:Eu²⁺ (CASN, SCASN)
- Properties:
  - Quantum yield: 80-90%
  - Peak: 610-650 nm
  - FWHM: 80-100 nm
- **Advantages**: Excellent thermal stability, deep red emission
- **Challenges**: Broadband emission, expensive

**K₂SiF₆:Mn⁴⁺** (KSF, Red):
- Narrow-band red phosphor
- Properties:
  - Quantum yield: 70-80%
  - Peak: 631 nm
  - FWHM: 30-40 nm (line emission)
- **Advantages**: Narrow linewidth, good color purity
- **Challenges**: Moisture sensitivity, particle size

**Advantages of Phosphors**:
- Mature technology (decades of use in lighting)
- Excellent thermal stability (>150°C)
- Long lifetime (>50,000 hours demonstrated)
- Lower cost than QDs currently

**Disadvantages**:
- Broader emission spectra (except KSF)
- Lower color purity than QDs
- Difficult to produce nanoparticles for thin films

### Organic Dyes

**Examples**: Rhodamine, perylene, coumarin derivatives

**Properties**:
- Quantum yield: 40-90%
- FWHM: 40-80 nm
- Easy solution processing

**Advantages**:
- Low cost
- Simple fabrication

**Disadvantages**:
- Poor photostability (photobleaching)
- Thermal instability
- Not suitable for long-lifetime displays

**Status**: Research only, not viable for commercial displays

## Color Conversion Architectures

### Chip-Level Conversion (On-Chip)

**Configuration**: Conversion material deposited directly on blue/UV LED chip before transfer

**Implementation**:
1. Fabricate blue LED wafer
2. Deposit green/red QD or phosphor layer on designated chips
3. Patterning and encapsulation
4. Transfer chips to display substrate

**Advantages**:
- Minimal optical crosstalk (conversion occurs at source)
- Thinner overall stack
- LED and converter tightly integrated
- Better heat dissipation (converter close to heat sink)

**Challenges**:
- High-temperature processing may damage QDs (requires low-temp process)
- Patterning difficulty at wafer level
- Three different chip types (blue, green-converted, red-converted) complicates transfer
- Rework difficult if conversion layer defective

**Status**: Used in some research prototypes; less common than panel-level conversion

### Panel-Level Conversion (Color Filter Replacement)

**Configuration**: Conversion layer deposited on TFT backplane in color filter position after LED transfer

**Implementation**:
1. Transfer blue LEDs to all pixel positions
2. Deposit conversion layers in red/green subpixel regions
3. Leave blue regions unconverted or with blue pass filter
4. Encapsulation and top cover glass

**Process**:

**Photo-Definable QD Resists**:
- QDs dispersed in photoresist
- Spin-coat and photolithography
- Pattern green and red regions
- No conversion in blue regions

**Inkjet Printing**:
- QD or phosphor ink formulations
- Inkjet print into red/green subpixels
- Advantages: Material efficiency, no photoresist waste
- Challenges: Uniformity, edge definition

**Photolithography + Blanket Coat**:
- Define wells or barriers
- Fill with QD/phosphor materials
- Doctor blade or slot-die coating

**Advantages**:
- Standard TFT process flow
- Familiar to display industry (similar to color filter fabrication)
- Easy to pattern and control thickness
- Defect repair easier (before converter deposition)

**Challenges**:
- Potential blue light leakage into adjacent subpixels
- Requires black matrix or barriers
- Optical losses from scattering and reflection

**Status**: Most common approach in MicroLED development

### Separate Conversion Layer (Laminated)

**Configuration**: Conversion material in separate film, laminated to display

**Implementation**:
1. Fabricate conversion film with QD or phosphor in polymer matrix
2. Pattern red/green regions
3. Align and laminate to display with blue LEDs

**Film Structure**:
- Polymer base (PET, polyimide, etc.)
- QD/phosphor layer
- Barrier layers (moisture/oxygen protection)
- Black matrix for color isolation
- Adhesive for lamination

**Advantages**:
- Conversion layer fabricated separately (optimized process)
- No high-temperature exposure to QDs during TFT processing
- Can be tested independently
- Potential for roll-to-roll manufacturing

**Challenges**:
- Alignment accuracy critical (<10 μm)
- Air gap increases optical crosstalk
- Thicker stack
- Lamination defects (bubbles, contamination)

**Status**: Used in some LCD-QD displays; possible for MicroLED

## Optical Design and Optimization

### Light Extraction from Conversion Layer

**Challenge**: Converted light emitted isotropically; half emits backward toward LED

**Solutions**:

**Reflective Layer Under Converter**:
- Metal mirror (silver, aluminum) reflects back-emitted light
- Efficiency gain: 40-60%
- Placement: Between LED and converter, or under conversion layer

**Scattering Particles**:
- TiO₂ or other high-index particles in conversion layer
- Scatter light to increase path length and extraction
- Optimal concentration: 1-10 wt%

**Photonic Structures**:
- Periodic nanostructures in/around conversion layer
- Engineer emission directionality
- Complex fabrication

### Crosstalk Reduction

**Black Matrix**:
- Opaque material between subpixels
- Absorbs scattered light
- Typical width: 5-20 μm
- Materials: Black photoresist, carbon black in polymer

**Barrier Structures**:
- Physical walls between subpixels
- Height: 5-30 μm
- Prevents lateral light propagation
- Fabricated by photolithography

**Polarization Control**:
- Not commonly used in MicroLED
- Potential for reducing scattered light

### Thickness Optimization

**Conversion Layer Thickness**:

**Too Thin**: Incomplete absorption, blue leakage
**Too Thick**: Excessive scattering, crosstalk, material cost

**Optimal Thickness** (depends on QD concentration and absorption):
- Green QD layer: 5-15 μm
- Red QD layer: 8-20 μm
- Phosphor layer: 20-50 μm (larger particle size)

**Calculation**:
Absorption follows Beer-Lambert law:
I(z) = I₀ × exp(-α × z)

Where:
- α = absorption coefficient (depends on QD concentration, cross-section)
- z = thickness
- Target: >95% absorption (z > 3/α)

### Encapsulation and Stability

**Degradation Mechanisms**:

**Photo-Oxidation**:
- Oxygen reacts with excited QDs
- Leads to loss of quantum yield
- Mitigation: Oxygen barrier layers, scavengers

**Moisture**:
- Especially critical for perovskite QDs
- Causes chemical decomposition
- Mitigation: Hermetic sealing, desiccants

**Thermal Degradation**:
- High temperatures accelerate degradation
- Especially for organic ligands on QDs
- Mitigation: Thermal management, high-stability materials

**Encapsulation Strategies**:

**Barrier Coatings**:
- SiOx, SiNx, Al₂O₃ by sputtering or ALD
- Water vapor transmission rate (WVTR): <10⁻⁶ g/m²/day target
- Oxygen transmission rate (OTR): <10⁻⁵ cm³/m²/day target

**Glass Encapsulation**:
- Thin glass lid sealed with frit or adhesive
- Excellent barrier properties
- Adds thickness and weight

**Hybrid Organic-Inorganic**:
- Alternating layers of organic and inorganic materials
- Defect decoupling improves barrier performance

**Lifetime Targets**:
- 50,000 hours at 60°C with <10% luminance loss
- Equivalent to 10-15 years typical display use

## Efficiency Analysis: Native RGB vs. Color Conversion

### System Efficiency Comparison

**Native RGB LEDs**:
- Blue LED EQE: 60%
- Green LED EQE: 45%
- Red LED (AlGaInP) EQE: 20% (at micro-scale)
- White light efficiency: Weighted average ~40% (depending on white point)

**Blue LED + QD Conversion**:
- Blue LED EQE: 60%
- Green conversion: 60% × 0.89 × 0.90 (QD QY) = 48%
- Red conversion: 60% × 0.74 × 0.85 (QD QY) = 38%
- White light efficiency: ~45-48% (depending on white point, color gamut)

**Conclusion**: Color conversion can match or exceed native RGB efficiency, particularly due to poor red LED performance at micro-scale

**Additional Considerations**:
- Native RGB has aperture losses (fill factor <100%)
- Color conversion has scattering losses and potential crosstalk
- Real-world efficiencies depend on specific implementation

### Color Gamut Comparison

**Native RGB MicroLED**:
- Blue: 450-465 nm (FWHM ~20 nm)
- Green: 520-530 nm (FWHM ~30 nm)
- Red: 620-630 nm (FWHM ~20 nm for AlGaInP)
- Color gamut: 85-95% Rec. 2020 typical

**Blue + InP QD Conversion**:
- Blue: 450-460 nm (FWHM ~20 nm)
- Green: 520-530 nm (FWHM ~35-45 nm)
- Red: 620-630 nm (FWHM ~35-45 nm)
- Color gamut: 90-95% Rec. 2020

**Blue + Perovskite QD** (if stability achieved):
- Narrower FWHM (~20-30 nm all colors)
- Color gamut: >95% Rec. 2020, approaching 100%

**Conclusion**: QD conversion can match or exceed native RGB gamut, especially if red InGaN LEDs replace AlGaInP

## Economic Considerations

### Cost Breakdown (Estimated)

**Native RGB Approach** (65" 4K display):
- Blue LED chips: 8.3M × $0.0005 = $4,150
- Green LED chips: 8.3M × $0.0006 = $4,980
- Red LED chips: 8.3M × $0.0012 = $9,960 (higher cost due to lower yield, AlGaInP)
- Transfer cost (3 types): $10,000-15,000
- **Total LED cost**: $29,000-34,000

**Blue + QD Conversion** (65" 4K display):
- Blue LED chips: 25M × $0.0004 = $10,000 (single type, volume economy)
- QD material (green): 0.8 m² × 10 μm × $500/L = $400
- QD material (red): 0.8 m² × 15 μm × $800/L = $960
- Deposition and patterning: $2,000-3,000
- Transfer cost (1 type): $5,000-8,000
- **Total cost**: $18,000-22,000

**Potential Savings**: 25-40% cost reduction with color conversion

**Caveats**:
- QD costs assumed at scale; current costs higher
- Native RGB costs assume mature manufacturing
- Transfer cost benefits assume simplified process
- Does not include potential efficiency differences affecting backlight requirements (not applicable to emissive displays)

### Manufacturing Complexity Trade-offs

**Color Conversion Advantages**:
- Single LED epitaxy and fabrication line
- Simplified transfer (one chip type)
- No RGB alignment required
- Smaller inventory (one chip type vs. three)

**Color Conversion Additional Steps**:
- QD synthesis or procurement
- Ink formulation
- Patterning process (inkjet or photolithography)
- Encapsulation (more critical than native RGB)
- Longer-term reliability validation

**Overall**: Color conversion trades LED complexity for materials and patterning complexity

## Future Directions

### InGaN Red LEDs

**Development Status**: Efficiency improving, 40-50% EQE demonstrated in labs

**Advantages**:
- Same material system as blue/green (manufacturing synergy)
- Better efficiency at micro-scale than AlGaInP
- Enables true native RGB GaN displays

**Challenges**:
- Requires high indium content (>35%) → strain, defects
- Wavelength control difficult
- Not yet production-ready

**Timeline**: Commercial viability 2026-2028 estimated

**Impact on Color Conversion**: If InGaN red succeeds, color conversion becomes less compelling for efficiency, but still offers manufacturing simplification

### Patterned Quantum Dot Films

**Emerging Technology**: Pre-patterned QD films for roll-to-roll manufacturing

**Process**:
1. Fabricate large-area QD film with red/green pattern
2. Laminate to MicroLED display
3. No need for QD patterning in display fab

**Advantages**:
- Separates QD processing from display manufacturing
- Enables specialized QD film manufacturers
- Potential for lower cost at scale

**Challenges**:
- Alignment accuracy
- Film handling and bonding
- Integration with varying pixel pitches

**Status**: R&D phase, partnerships forming between QD and film manufacturers

### Perovskite Quantum Dot Stabilization

**Research Focus**: Improving stability of perovskite QDs to match InP

**Approaches**:
- Core/shell structures with protective layers
- Encapsulation in inorganic matrices (e.g., silica)
- Ligand engineering for moisture resistance
- Lead-free perovskite alternatives (e.g., Cs₃Bi₂Br₉)

**Potential**: If stability achieved, perovskite QDs could become dominant due to superior color purity

**Timeline**: 3-5 years to commercial viability (optimistic)

### AI-Optimized Color Conversion

**Application**: Machine learning for conversion layer design

**Optimization Parameters**:
- QD concentration
- Layer thickness
- Scattering particle type and concentration
- Black matrix geometry

**Benefits**:
- Maximized efficiency
- Minimized crosstalk
- Optimized color uniformity

**Status**: Early research demonstrations

## Conclusion

Color conversion represents a viable and potentially superior alternative to native RGB LEDs for MicroLED displays. The choice between native RGB and color conversion depends on:

**Favor Native RGB when**:
- Maximum efficiency is critical
- InGaN red LEDs mature
- Application justifies RGB complexity (ultra-premium displays)

**Favor Color Conversion when**:
- Manufacturing simplification priority
- Cost reduction critical
- Red LED efficiency is limited (current AlGaInP technology)
- Widest color gamut required (with best QDs)

Current industry trend: Large format displays (Samsung The Wall) use native RGB due to easier large chip handling. Future smartphones and tablets may adopt color conversion for cost and manufacturability benefits.

The WIA-SEMI-010 standard defines color conversion materials, performance metrics, patterning requirements, and testing protocols to enable consistent industry practices and facilitate development of this critical technology.

---

**Key Color Conversion Metrics:**
- QD quantum yield: >85% (green), >80% (red) target
- Conversion efficiency: >75% (blue→green), >65% (blue→red)
- Color gamut: >90% Rec. 2020
- Lifetime: >50,000 hours @ 60°C with <10% loss
- Cost target: <$10 per 65" display at scale

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
