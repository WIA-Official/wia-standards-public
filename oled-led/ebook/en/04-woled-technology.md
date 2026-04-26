# Chapter 4: WOLED Technology

## White OLED Architecture and LG's Market-Leading Approach

White OLED (WOLED) technology represents LG Display's strategic answer to the challenge of manufacturing large-area OLED displays economically. This chapter explores the technical details, advantages, trade-offs, and optimization strategies of WOLED technology.

### WOLED Architecture Fundamentals

#### Basic Structure

Unlike RGB OLED which creates colors by depositing separate red, green, and blue emissive materials through fine metal masks, WOLED uses a simpler approach: create white light from the OLED stack, then filter it through color filters to create RGB subpixels.

**Complete WOLED Stack (Bottom to Top):**

1. **Glass Substrate** (0.7mm typical)
   - TFT glass with circuit traces
   - Can be up to 2500mm diagonal (Gen 8.5)

2. **TFT Backplane** (LTPS or Oxide TFT)
   - Thin-film transistors control each subpixel
   - LTPS (Low-Temperature Polysilicon): Higher electron mobility
   - Oxide TFT: Better uniformity, lower cost for large area

3. **Planarization Layer**
   - Smooths surface over TFT circuitry
   - Critical for uniform OLED deposition

4. **Anode (ITO - Indium Tin Oxide)**
   - Transparent conductive layer
   - Thickness: 100-150nm
   - Acts as hole injection electrode

5. **OLED Emission Layers**
   - This is the key innovation of WOLED
   - Multiple emission layers creating white light
   - Total organic stack thickness: ~100-150nm

6. **Cathode (Reflective Metal)**
   - Aluminum or silver
   - Reflects light back through anode
   - Top emission configuration

7. **Encapsulation**
   - Thin-film encapsulation (TFE) or glass seal
   - Critical: protects organics from oxygen and moisture
   - Modern TFE: alternating organic/inorganic layers

8. **Color Filter Array**
   - Red, Green, Blue, White (RGBW) pattern
   - Absorbs unwanted wavelengths
   - Critical for color purity

9. **Polarizer & Anti-Reflection Layers**
   - Reduces ambient light reflection
   - Improves contrast in bright environments
   - Circular polarizer typical

### White Light Generation Methods

**Method 1: Stacked RGB Emission (LG's Primary Approach)**

Stack multiple emission layers within the OLED structure:
- Blue emission layer (shortest wavelength, highest energy)
- Green emission layer
- Red emission layer
- Combined emission creates white light

**Advantages:**
- High color rendering index (CRI >90)
- Broad spectrum white light
- Good efficiency balance

**Challenges:**
- Complex stack engineering
- Balancing emission from each layer
- Efficiency trade-offs between colors

**Method 2: Blue + Yellow Emission**

Simpler two-layer approach:
- Blue OLED emission layer
- Yellow OLED emission layer (or yellow phosphorescent dopant)
- Blue + Yellow perceived as white

**Advantages:**
- Simpler stack structure
- Potentially higher efficiency
- Easier manufacturing

**Challenges:**
- Narrower spectrum
- Lower CRI than RGB stacking
- May require more aggressive color filtering

**LG's Implementation:**

LG uses a sophisticated multi-layer tandem structure:
- Bottom emission unit: Blue and yellow emitters
- Middle charge generation layer (allows stacking)
- Top emission unit: Green and red emitters
- Result: Broad-spectrum white light with high efficiency

This tandem approach allows higher brightness without increasing current density proportionally, reducing OLED degradation.

### RGBW Subpixel Configuration

**Subpixel Pattern:**

LG's WOLED uses four subpixels per pixel:
- Red (with red color filter over white OLED)
- Green (with green color filter over white OLED)
- Blue (with blue color filter over white OLED)
- White (no color filter, white OLED passes through)

**Why Add White Subpixel?**

1. **Brightness Boost:**
   - Color filters absorb 60-70% of light
   - White subpixel has no filter, passes all light
   - Significantly increases peak luminance capability

2. **Efficiency Improvement:**
   - White content (common in UI, text) uses efficient W subpixel
   - Reduces power consumption for bright scenes
   - Extends OLED lifetime (less current needed for same brightness)

3. **Manufacturing Yield:**
   - White OLED uniformity easier to achieve than RGB
   - Defects in one color don't require pixel repair
   - Higher panel yield

**Subpixel Arrangement Pattern:**

Various patterns used:
- RGBW stripes (simple, lower resolution)
- RGBW quad (2x2 arrangement)
- Optimized patterns for specific resolutions

For 4K panel:
- Still maintains 3840x2160 RGB resolution
- W subpixels added without reducing RGB count
- Some configurations share subpixels between adjacent pixels

### Color Filter Design

**Critical Component:**

Color filters in WOLED are more critical than in LCD:
- Must achieve high color purity from broad white spectrum
- Efficiency directly impacts overall system efficacy
- Durability important (subject to OLED heat)

**Filter Specifications:**

**Red Filter:**
- Pass band: 580-700nm (red wavelengths)
- Block: <580nm (block green, blue)
- Transmission in pass band: 80-90%
- Out-of-band rejection: >95%

**Green Filter:**
- Pass band: 500-580nm
- Block: <500nm and >580nm
- Transmission: 75-85% in pass band
- Critical for image quality (human eye most sensitive to green)

**Blue Filter:**
- Pass band: 420-500nm
- Block: >500nm
- Transmission: 70-80%
- Most challenging (blue OLEDs naturally less efficient)

**White Subpixel:**
- No filter, or neutral density filter for uniformity
- Maximum transmission
- May include scattering layer for diffusion

**Manufacturing:**

- Photolithography process (similar to LCD color filters)
- Pigment-based or dye-based filters
- Typically 1-2μm thick
- Applied to TFT substrate or separately laminated

### Manufacturing Process Flow

**Step 1: TFT Backplane Fabrication** (8-12 masks)
- Deposit and pattern semiconductor layers
- Create transistor structures
- Form electrical connections
- Similar to LCD TFT process (leverage existing expertise)

**Step 2: Planarization**
- Deposit organic planarization layer
- Chemical-mechanical polishing if needed
- Surface roughness <5nm required

**Step 3: ITO Anode Deposition**
- Sputter ITO layer
- Pattern into individual subpixel anodes
- Oxygen plasma treatment for work function tuning

**Step 4: Organic Layer Deposition** (In Vacuum)
- Load substrate into vacuum chamber (<10^-6 torr)
- Heat evaporation sources for each organic material
- Deposit layers sequentially:
  - Hole injection layer (HIL)
  - Hole transport layer (HTL)
  - Emission layers (multi-layer for white)
  - Electron transport layer (ETL)
  - Electron injection layer (EIL)
- Critical: no masking needed (unlike RGB OLED)
- Full-surface deposition dramatically improves yield and speed

**Step 5: Cathode Deposition**
- Deposit reflective cathode (Al or Ag)
- Thickness: 100-150nm
- Some patterning may be needed for electrical isolation

**Step 6: Encapsulation**
- Option A: Thin-film encapsulation (TFE)
  - Alternate organic and inorganic layers (5-7 layers)
  - Total thickness: <5μm
  - Enables thinner, lighter displays
- Option B: Glass frit seal
  - Attach cover glass with perimeter seal
  - Thicker but very reliable
  - Desiccant inside sealed cavity

**Step 7: Color Filter Application**
- Option A: Deposit on TFT substrate before OLED
  - Color filters on bottom, OLED on top
  - Requires precise alignment
- Option B: Separate CF glass, laminated to OLED panel
  - CF array on thin glass
  - Optical adhesive lamination
  - Allows separate optimization

**Step 8: Polarizer and Final Lamination**
- Circular polarizer lamination
- Anti-reflection coatings
- Cover glass attachment

**Step 9: Testing and Binning**
- Electrical testing (all pixels)
- Optical testing (luminance, color, uniformity)
- Aging test (burn-in compensation data)
- Binning by quality grade

### WOLED Performance Characteristics

**Brightness:**

**Full White (100% APL):**
- Standard WOLED: 150-200 Cd/m²
- WOLED EVO (latest): 200-300 Cd/m²
- Limited by thermal management and lifetime concerns

**Peak White (10% Window):**
- Standard WOLED: 600-750 Cd/m²
- WOLED EVO: 800-1,000 Cd/m²
- MLA (Micro Lens Array) version: 900-1,100 Cd/m²

**HDR Highlights (3% Window):**
- Can achieve 1,000+ Cd/m² in small areas
- ABL (Automatic Brightness Limiter) controls overall power
- Sufficient for HDR10, adequate for Dolby Vision

**Color Performance:**

**Color Gamut:**
- DCI-P3 coverage: 95-98% (very good)
- Rec.2020 coverage: 70-75% (adequate)
- sRGB: 100%+ (over-saturated if not calibrated)

**Limiting Factors:**
- White OLED spectrum broader than RGB OLED
- Color filters cannot perfectly isolate narrow bands
- Trade-off: purity vs efficiency

**Color Accuracy:**
- Delta E <2 achievable with calibration
- LG's TV processors do excellent calibration
- Professional calibration can reach Delta E <1

**Color Volume:**
- Excellent at low luminance levels (OLED advantage)
- Good at mid luminance
- Limited at very high luminance (vs Mini-LED)
- Overall very competitive

**Efficiency:**

**Luminous Efficacy:**
- White OLED emission: 80-120 lm/W
- After color filters: 25-40 lm/W effective
- Complete system (including drivers, scaler): 20-35 lm/W

**Power Consumption (55" TV typical):**
- Watching dark movie: 50-80W
- Watching bright movie: 100-150W
- Displaying full white: 200-250W
- HDR highlights: varies, ABL limits maximum

**Efficiency Comparison:**
- Better than RGB OLED (no FMM, better uniformity)
- Comparable to or better than Mini-LED for dark content
- Worse than LED LCD for bright/white content

**Lifetime:**

**LT95 (Time to 95% initial brightness):**
- Standard WOLED: 50,000 hours (continuous max brightness)
- Realistic usage: 100,000+ hours
- Much longer with compensation algorithms

**Degradation Characteristics:**
- Blue OLED degrades fastest (always the challenge)
- Red and green more stable
- Compensation cycles adjust voltage to maintain brightness
- Eventual color shift as compensation reaches limits

**Burn-in:**
- Static content can cause uneven aging
- Modern panels have multiple mitigation strategies:
  - Pixel shift (move image slightly)
  - Logo detection and dimming
  - Pixel refresher cycles
  - Voltage compensation
- Risk low with varied content, higher with static UI

### WOLED Optimization Technologies

**Micro Lens Array (MLA):**

LG's major innovation in WOLED EVO:

**Concept:**
- Microscopic lens array on top of OLED emission
- ~5-10μm lenses, billions per panel
- Focus and redirect light toward viewer

**Benefits:**
- Increases light extraction efficiency by 20-30%
- Same OLED brightness → more light out
- Or: less OLED drive current → same light out
  - Reduced power consumption
  - Extended OLED lifetime
  - Lower operating temperature

**Implementation:**
- Lens array created by lithography
- Precisely aligned to subpixel pattern
- Integrated into encapsulation or polarizer

**Results:**
- WOLED EVO with MLA: 800-1,000 Cd/m² peak (vs 600-700 without)
- 20-25% power reduction at same brightness
- Significant competitive advantage

**Tandem WOLED:**

LG developing tandem structure for even better performance:

**Architecture:**
- Two OLED emission units in series
- Charge generation layer between them
- Each unit operates at lower current density

**Benefits:**
- 2x lifetime (same brightness, half current per unit)
- Or: 2x brightness (same current, two units emitting)
- Better efficiency curve
- More stable color over lifetime

**Challenges:**
- More complex manufacturing
- Higher voltage required
- Increased stack thickness (minor)

**Status:**
- Demonstrated in research
- Small-scale production for automotive (2024-2025)
- Large-scale TV production: 2025-2027 expected

**Advanced TFT Backplanes:**

**LTPO (Low-Temperature Polycrystalline Oxide):**
- Hybrid TFT technology
- LTPS for switching, oxide for storage
- Enables very low power standby
- Variable refresh rate (1Hz to 120Hz)
- Initially for mobile, coming to TV

**Benefits for WOLED:**
- Lower power consumption
- Smoother video (variable refresh)
- Better gaming performance

### WOLED vs QD-OLED Direct Comparison

| Aspect | WOLED | QD-OLED |
|--------|-------|---------|
| **Manufacturing Complexity** | Simpler (no FMM) | More complex (QD integration) |
| **Manufacturing Yield** | Higher | Lower (currently) |
| **Manufacturing Cost** | Lower | Higher |
| **Peak Brightness** | 800-1,000 Cd/m² | 1,000-1,500 Cd/m² |
| **Color Gamut** | 95% DCI-P3 | 99% DCI-P3 |
| **Color Purity** | Good | Excellent |
| **Efficiency** | Moderate (filters lose light) | Better (less filter loss) |
| **Viewing Angle** | Excellent | Very good (slight variation) |
| **Maturity** | Mature (10+ years) | New (2-3 years) |
| **Availability** | Wide (many brands) | Limited (Samsung/Sony) |
| **Price** | Decreasing, now <$2k for 55" | Premium, $2.5k+ for 55" |

**Market Positioning:**
- WOLED: Mainstream premium to mid-premium OLED
- QD-OLED: Ultra-premium, emphasizing color and brightness

Both are excellent technologies; choice depends on priorities and budget.

### Future WOLED Developments

**2025-2026:**
- Tandem WOLED entering production
- Further MLA optimization
- New emitter materials for better blue efficiency
- Cost reduction targeting <$1,500 for 55"

**2027-2030:**
- Tandem WOLED mainstream
- 2,000 Cd/m² peak brightness achievable
- 100,000+ hour lifetime at full brightness
- Expansion to monitor and commercial display markets
- Larger sizes (100"+ more affordable)

**Beyond 2030:**
- Depends on competition from QD-OLED, Micro-LED
- Possible consolidation or technology shift
- WOLED may evolve or be succeeded

### Conclusion

WOLED technology represents an elegant engineering solution to OLED manufacturing challenges:

**Key Advantages:**
- Simpler manufacturing than RGB OLED
- Higher yield and lower cost than alternatives
- Scalable to large sizes (proven up to 88", capable of larger)
- Continuous improvement demonstrating mature technology path
- Wide availability from multiple TV brands

**Key Limitations:**
- Efficiency penalty from color filters
- Slightly limited color gamut vs QD approaches
- Peak brightness lower than best QD-OLED or Mini-LED
- Still has OLED burn-in risk (though well-managed)

**Market Reality:**
- WOLED dominates OLED TV market with >60% share
- LG's investment and scale difficult for competitors to match
- Price-performance ratio improving yearly
- Technology continues advancing (MLA, tandem, etc.)

For consumers seeking OLED TVs, WOLED offers proven quality at increasingly accessible prices. For engineers and product planners, WOLED represents a masterclass in balancing performance, manufacturability, and cost.

---

**Next Chapter**: QD-OLED Technology - Samsung's quantum dot color conversion approach, offering superior color gamut and brightness at premium positioning.
