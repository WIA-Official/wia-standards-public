# Chapter 5: QD-OLED Technology

## Quantum Dot Color Conversion for Ultimate Color Performance

QD-OLED (Quantum Dot Organic Light-Emitting Diode) represents Samsung Display's innovative approach to large-area OLED manufacturing, combining blue OLED emission with quantum dot color conversion to achieve exceptional color gamut and brightness. This chapter explores the science, engineering, and market implications of this emerging technology.

### Quantum Dot Fundamentals

#### What Are Quantum Dots?

Quantum dots (QDs) are semiconductor nanocrystals typically 2-10 nanometers in diameter - so small that quantum mechanical effects dominate their behavior.

**Key Quantum Mechanical Property:**
- Electron confinement in three dimensions
- Creates discrete energy levels (like atoms)
- Energy levels determined by QD size
- Smaller QD = larger bandgap = higher energy (bluer) emission
- Larger QD = smaller bandgap = lower energy (redder) emission

**Photoluminescent Conversion:**
- QDs absorb high-energy (blue) light
- Electrons excited to higher energy state
- Electrons relax, emitting lower-energy (red or green) light
- Emission wavelength precisely controlled by QD size
- Very narrow emission spectrum (30-40nm FWHM)

**Materials:**

**Cadmium-based (Traditional):**
- CdSe (Cadmium Selenide) core
- ZnS (Zinc Sulfide) shell for stability
- Excellent optical properties
- Environmental concerns (cadmium toxicity)
- Restricted in some markets (EU RoHS)

**Cadmium-free (Emerging):**
- InP (Indium Phosphide) core
- ZnS or ZnSe shell
- Environmentally friendly
- Performance approaching Cd-QDs
- Higher cost currently

**Samsung's QD-OLED likely uses:**
- Cd-free QDs for regulatory compliance
- Highly optimized for blue light absorption
- Encapsulated to prevent degradation

### QD-OLED Architecture

#### Complete Stack (Bottom Emission)

**1. Glass Substrate**
- Standard TFT glass
- Gen 8.5 (2200x2500mm) capability

**2. TFT Backplane (LTPS)**
- Low-Temperature Polysilicon TFTs
- High electron mobility needed for OLED
- Pixel driver circuits integrated

**3. OLED Stack - Blue Emission**
- Anode (ITO)
- Hole injection/transport layers
- **Blue emissive layer** (this is critical)
  - Fluorescent or phosphorescent blue emitter
  - Must emit at ~450-460nm (deep blue)
  - High efficiency required
  - Long lifetime essential
- Electron transport/injection layers
- Cathode (semi-transparent)

**4. Quantum Dot Color Conversion Layer**
- Red QDs over "red" pixels (converts blue→red)
- Green QDs over "green" pixels (converts blue→green)
- No QDs over "blue" pixels (blue passes through)
- Critical: precise patterning required
- Thickness: 5-10μm typical

**5. Color Filter Array (Optional but typical)**
- Red filter over red QDs (blocks any unconverted blue)
- Green filter over green QDs
- Blue filter over blue areas
- Improves color purity and contrast

**6. Encapsulation**
- Protects QDs and OLED from moisture/oxygen
- Thin-film encapsulation typical
- Critical for QD stability

**7. Polarizer/Anti-Reflection**
- Circular polarizer
- Reduces ambient reflection
- Improves perceived contrast

#### Why Blue OLED Base?

**Advantages of Blue Base:**

1. **Manufacturing Simplicity**
   - No fine metal mask needed for color patterning
   - Full-surface blue OLED deposition
   - Higher yield than RGB OLED
   - QD patterning simpler than OLED patterning

2. **Quantum Dot Efficiency**
   - QDs convert high energy (blue) → lower energy (red/green)
   - Downconversion thermodynamically favorable
   - QD conversion efficiency 60-80%
   - Upconversion (red→blue) would be inefficient

3. **Blue OLED Challenges Met Differently**
   - Blue OLED lifetime always worst
   - But in QD-OLED, all pixels use same blue OLED
   - Uneven aging problem of RGB OLED avoided
   - Can optimize single blue emitter aggressively

**Challenges of Blue Base:**

1. **Blue OLED Efficiency**
   - Blue OLEDs less efficient than red/green
   - Must overcome QD conversion losses
   - Requires very good blue emitter

2. **Blue OLED Lifetime**
   - Shortest lifetime of OLED colors
   - Determines entire display lifetime
   - Significant R&D required

3. **QD Stability**
   - Must remain stable under blue light exposure
   - Heat from OLED can degrade QDs
   - Requires robust encapsulation

### Quantum Dot Patterning

**Critical Manufacturing Step:**

Creating precise red/green/blue pattern with QDs is challenging:

**Method 1: Inkjet Printing**
- Dispense QD solution as "ink"
- Precisely deposit QDs in pixel areas
- Red QD ink in red pixels, green in green
- Advantages: Direct patterning, material efficiency
- Challenges: Uniformity, drying, cross-contamination

**Method 2: Photolithography**
- Coat full surface with QD-containing photoresist
- Expose pattern with UV light
- Develop to remove unexposed areas
- Repeat for each color
- Advantages: Precise patterns, proven technique
- Challenges: QD damage from solvents/UV

**Method 3: Transfer Printing**
- Pattern QDs on separate substrate
- Transfer to display substrate
- Advantages: QDs protected from harsh processing
- Challenges: Complex equipment, alignment

**Samsung's Approach:**
- Likely hybrid of multiple techniques
- Proprietary process, not publicly detailed
- Achieving high yield and uniformity
- Continuous improvement ongoing

### Color Performance Analysis

#### Color Gamut

QD-OLED achieves industry-leading color gamut:

**Measured Performance:**
- DCI-P3: 98-100% coverage
- Rec.2020: 85-90% coverage
- Adobe RGB: ~100%
- sRGB: 100%+ (over-saturated if not managed)

**Comparison:**
- Best WOLED: ~95% DCI-P3, 70% Rec.2020
- Best Mini-LED (with QD film): 95-98% DCI-P3, 75-80% Rec.2020
- QD-OLED clear leader

**Why QD-OLED Wins Color Gamut:**

1. **Narrow Emission Spectra**
   - QDs emit very pure colors (30-40nm FWHM)
   - Red QDs: Peak ~630nm, minimal orange/yellow
   - Green QDs: Peak ~530nm, minimal yellow/cyan
   - Blue OLED: ~450-460nm, pure deep blue

2. **Optimized Primary Points**
   - Can tune QD size to hit ideal primary locations
   - Closer to Rec.2020 primaries than other technologies
   - Green especially pure (often limiting factor)

3. **Minimal Filter Loss**
   - QDs already provide narrow spectrum
   - Color filters mainly block unconverted blue
   - Less aggressive filtering needed vs WOLED

#### Color Volume

Color volume = color gamut × luminance range

**QD-OLED Performance:**
- Excellent at all luminance levels
- Color saturation maintained at high brightness
- DCI-P3 coverage >95% even at 1,000 Cd/m²
- Better than WOLED (filters limit bright color saturation)
- Competitive with or better than Mini-LED

**Measured Results (Sony A95K QD-OLED):**
- 100% DCI-P3 at 500 Cd/m²
- 98% DCI-P3 at 1,000 Cd/m²
- 95% DCI-P3 at peak brightness
- Superior color volume to most competitors

### Brightness Performance

#### Peak Brightness

**Current QD-OLED (2024-2025):**
- Full screen (100% APL): 200-300 Cd/m²
- Large window (50% APL): 500-700 Cd/m²
- Medium window (25% APL): 800-1,000 Cd/m²
- Small window (10% APL): 1,000-1,500 Cd/m²
- Highlights (3% APL): 1,500-2,000 Cd/m² peak

**Advantages vs WOLED:**
- 20-30% higher peak brightness
- Better sustained brightness (less ABL limiting)
- Improved HDR highlight rendering

**Limitations vs Mini-LED:**
- Still below best Mini-LED (3,000-5,000 Cd/m²)
- ABL still present (thermal/lifetime management)
- Full-screen brightness limited

**Why QD-OLED Brighter Than WOLED:**

1. **Efficiency Path:**
   - Blue OLED → QD conversion more efficient than
   - White OLED → color filter approach
   - Less light absorbed

2. **Optimized Blue Emitter:**
   - Samsung focused solely on best blue OLED
   - Years of smartphone OLED expertise applied
   - Continuous refinement

3. **Reduced Filter Loss:**
   - Color filters less aggressive
   - Blue pixels especially (no conversion needed)
   - Overall transmission higher

#### Brightness Uniformity

**Potential Concern:**
- Blue OLED uniformity easier than RGB
- But QD conversion adds variability
- QD layer thickness variations
- QD concentration variations

**Samsung's Solutions:**
- Precise QD patterning process
- Compensation in TFT driving
- Aging cycles to stabilize
- Testing and binning

**Measured Results:**
- Modern QD-OLED very uniform
- ±5% across panel typical (excellent)
- Not worse than WOLED in practice

### Viewing Angle Characteristics

**Unique QD-OLED Behavior:**

Early QD-OLEDs (2022-2023) showed slight luminance reduction at wide viewing angles:
- ~20-30% brightness reduction at 60° from center
- Color shift minimal (OLED advantage retained)
- More noticeable than WOLED
- Attributed to QD emission directivity

**Causes:**
- QDs emit light in all directions
- Some light trapped by total internal reflection
- Extraction efficiency varies with viewing angle
- Blue OLED also has some directivity

**Solutions Applied (2024+):**
- Optimized QD layer refractive index
- Micro-structured surfaces to improve extraction
- Better optical coatings
- Significant improvement in newer panels

**Current Status:**
- 2024-2025 QD-OLED much improved
- Viewing angle performance competitive with WOLED
- Still slight reduction but less noticeable
- Acceptable for typical viewing scenarios

### Efficiency Analysis

#### Luminous Efficacy Path

**Blue OLED Efficiency:**
- Modern blue OLED: 30-50 lm/W (challenging)
- Less efficient than red/green OLEDs
- Improving with new materials

**QD Conversion Efficiency:**
- Blue → Red: 60-70% quantum efficiency
- Blue → Green: 70-80% quantum efficiency
- Losses from:
  - Stokes shift (energy difference)
  - Some absorption without re-emission
  - Total internal reflection

**Overall System:**
- Blue OLED: 30-50 lm/W
- After QD conversion: 20-40 lm/W (effective)
- After color filters: 18-35 lm/W
- Complete system: 15-30 lm/W

**Comparison:**
- Better than WOLED: 20-30 lm/W system
- Worse than LED LCD: 100-150 lm/W
- Similar to RGB OLED

#### Power Consumption

**55" QD-OLED TV Typical:**
- Dark movie content: 60-100W
- Mixed content: 120-180W
- Bright content: 180-250W
- Full white (testing): 300-350W

**Comparison to WOLED:**
- Slightly higher power for same content
- But achieves higher peak brightness
- Trade-off: more brightness capability for similar power

### Lifetime and Reliability

#### Expected Lifetime

**Current Generation:**
- Rated lifetime: 50,000-70,000 hours (LT95)
- Conservative estimate: matches WOLED
- Dependent on blue OLED degradation

**Limiting Factor: Blue OLED**
- Blue emitter degrades fastest (always true for OLED)
- Determines entire display lifetime
- All pixels age similarly (advantage vs RGB OLED)

**QD Stability:**
- Modern QDs quite stable under blue light
- Encapsulation protects from oxygen/moisture
- QD degradation not primary concern
- Blue OLED degrades before QDs fail

#### Burn-in Characteristics

**Risk Level:**
- Similar to other OLED technologies
- Static content can cause uneven aging
- Blue OLED all pixels = uniform aging helps
- Still need mitigation strategies

**Mitigation Techniques (Applied in QD-OLED TVs):**
1. Pixel shift (slight image movement)
2. Logo detection and dimming
3. Screen savers for static elements
4. Periodic refresh/compensation cycles
5. Voltage compensation for aged areas

**Practical Risk:**
- Low with varied content (movies, gaming)
- Moderate with mixed content (some UI)
- Higher with constant static elements (tickers, logos)
- Similar to WOLED in practice

### Manufacturing Challenges

#### Blue OLED Requirements

**Very Demanding:**
- Must be highly efficient (overcome conversion losses)
- Must have long lifetime (determines display life)
- Must be uniform across large area
- Must maintain performance at high drive current

**Samsung's Advantage:**
- Decades of blue OLED R&D for smartphones
- World's best blue OLED materials and processes
- Continuous improvement

**Challenge:**
- Blue OLED remains difficult
- Trade-off between efficiency, brightness, lifetime
- Ongoing materials research critical

#### QD Integration

**Complex Process:**
- QD patterning precision required
- Three separate QD depositions (or complex process)
- Uniformity critical
- Must protect QDs during subsequent processing

**Yield Impact:**
- More complex than WOLED (no color patterning)
- Less complex than RGB OLED (no FMM)
- Current yield: competitive but improving
- Cost impacted by lower yield vs WOLED

#### Scale-up Challenges

**Capacity:**
- Samsung's Q1 line: 30,000 Gen 8.5 sheets/month
- Expanding capacity (Q2 line planned)
- Currently supply-limited

**Cost:**
- Higher than WOLED currently
- Decreasing with volume and learning curve
- Target: match WOLED cost in 3-5 years

### QD-OLED vs WOLED Technical Summary

| Parameter | QD-OLED | WOLED |
|-----------|---------|-------|
| **Color Gamut** | 98-100% DCI-P3 | 95-98% DCI-P3 |
| **Peak Brightness** | 1,000-1,500 Cd/m² | 800-1,000 Cd/m² |
| **Efficiency** | 18-30 lm/W | 20-30 lm/W |
| **Manufacturing** | More complex | Simpler |
| **Cost** | Higher | Lower |
| **Viewing Angle** | Very good (improving) | Excellent |
| **Lifetime** | 50,000-70,000h | 50,000-100,000h |
| **Maturity** | New (2-3 years) | Mature (10+ years) |
| **Availability** | Limited (Samsung/Sony) | Wide (many brands) |

### Future QD-OLED Roadmap

#### Near-Term (2025-2026)

**Improvements:**
- Brighter blue OLED (better materials)
- Peak brightness → 2,000 Cd/m²
- Improved QD stability
- Better viewing angle coating
- Cost reduction (increasing volume)

**Market Expansion:**
- More screen sizes (currently limited)
- Possible 42" for monitors
- 77" and larger expansion
- Additional TV brands adopting (beyond Sony)

#### Medium-Term (2027-2029)

**Tandem QD-OLED:**
- Two-stack blue OLED
- 2x brightness or 2x lifetime
- Potentially 3,000+ Cd/m² peak
- 100,000+ hour lifetime
- Premium positioning

**QD Material Advances:**
- Better QD formulations
- Higher conversion efficiency
- Improved stability
- Possibly perovskite QDs

**Manufacturing:**
- Gen 10.5 fab potential (larger sheets)
- Higher throughput
- Cost approaching WOLED

#### Long-Term (2030+)

**Potential Paths:**

1. **QD-OLED Dominant:**
   - If cost parity with WOLED achieved
   - Superior color gamut wins market
   - LG might adopt QD-OLED

2. **Micro-LED Transition:**
   - If Micro-LED costs drop sufficiently
   - QD-OLED might be transitional technology
   - 5-10 year window before replacement

3. **QD-Electroluminescent:**
   - Direct QD emission (no OLED)
   - Would supersede QD-OLED
   - Currently far from commercialization

### Conclusion

**QD-OLED Strengths:**
- Industry-leading color gamut and purity
- Higher brightness than WOLED
- Excellent contrast (OLED inherent)
- Fast response time (OLED inherent)
- Exceptional image quality

**QD-OLED Limitations:**
- Higher cost than WOLED (currently)
- Limited availability
- Slightly worse viewing angle (improving)
- Manufacturing complexity

**Market Position:**
- Ultra-premium segment
- Tech enthusiasts and videophiles
- Competes with best WOLED and Mini-LED
- Differentiation through color performance

**Recommendation:**
- Choose QD-OLED if: Prioritize ultimate color, willing to pay premium, want cutting-edge technology
- Choose WOLED if: Want excellent OLED at better value, proven technology, wider selection
- Choose Mini-LED if: Prioritize brightness, no burn-in risk, bright room use

QD-OLED represents Samsung's ambitious entry into large OLED, leveraging their quantum dot and OLED expertise to create a technologically impressive product that pushes the boundaries of display performance.

---

**Next Chapter**: Tandem OLED Technology - The future of OLED with stacked architectures promising doubled lifetime and brightness.
