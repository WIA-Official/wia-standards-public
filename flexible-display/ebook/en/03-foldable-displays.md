# Chapter 3: Foldable Display Technology

## Engineering the Fold: From Concept to Consumer Reality

---

### 3.1 Introduction to Foldable Displays

Foldable displays represent the most commercially successful category of flexible display technology, with millions of units shipped annually by manufacturers including Samsung, Huawei, Motorola, Xiaomi, OPPO, and others. These devices offer a compelling value proposition: the portability of a smartphone with the screen real estate of a tablet.

#### The Foldable Revolution Timeline

**2011**: Samsung demonstrates prototype foldable AMOLED at CES
**2018**: Royole FlexPai - first commercial foldable smartphone (limited availability)
**2019**: Samsung Galaxy Fold launches (February announcement, September release after redesign)
**2019**: Huawei Mate X announced (out-folding design)
**2020**: Motorola Razr revival with foldable display, Samsung Galaxy Z Flip (clamshell form factor)
**2021**: Samsung Galaxy Z Fold 3 with under-display camera, IPX8 water resistance
**2022**: Ultra-thin glass (UTG) becomes standard, crease reduction improvements
**2023**: Tri-fold and slide-out concepts demonstrated
**2024**: Mainstream adoption accelerates, prices drop below $1000
**2025**: WIA-SEMI-011 standard establishes industry benchmarks

### 3.2 Fold Configurations

#### In-Folding (Valley Fold)

**Description:** Display bends inward, screen faces inside when folded

**Advantages:**
- Protected screen when closed (no scratches from keys, coins)
- Hinge can be simpler (less visible gap)
- Easier to achieve small bend radius
- More compact when folded

**Disadvantages:**
- Requires external cover display (adds cost, complexity)
- Cannot use device in folded state (except cover screen)
- Hinge more visible on back when open
- Dust ingress risk at hinge

**Examples:**
- Samsung Galaxy Z Fold series (2-5)
- Huawei Mate X2, X3
- OPPO Find N series
- Honor Magic V series
- Xiaomi Mix Fold series

**Design Considerations:**

Bend Radius for In-Folding:
- **UTG**: 1.5-3mm minimum radius
- **Polymer (PI)**: 1-2mm achievable
- **Stress Distribution**: Compression on inside, tension on outside

Optimal radius = 2.5mm (balance of stress, hinge compactness, crease visibility)

#### Out-Folding (Mountain Fold)

**Description:** Display bends outward, screen exposed on both sides when folded

**Advantages:**
- No need for separate cover display (cost savings)
- Always-accessible screen
- Seamless user experience
- Larger effective screen area

**Disadvantages:**
- Screen vulnerable to damage when folded
- Larger bend radius needed (4-6mm typical)
- More complex hinge mechanism
- Visible gap when closed
- Higher screen replacement costs due to exposure

**Examples:**
- Huawei Mate X (original, 2019)
- Huawei Mate Xs, Xs 2
- Royole FlexPai 1, 2

**Design Considerations:**

Bend Radius for Out-Folding:
- **UTG**: 3-5mm minimum (higher stress on outer surface)
- **Polymer**: 2-4mm achievable
- **Protection**: Requires extremely durable cover glass or coating

Protective Strategies:
- Hard coating layers (7H+ pencil hardness)
- Sacrificial outer layers that can be replaced
- Frame design that elevates screen above surface when folded

#### Clamshell (Vertical Fold)

**Description:** Folds vertically like traditional flip phones

**Advantages:**
- Compact when folded (pocket-friendly)
- Nostalgic form factor with modern technology
- Lower cost (smaller display area)
- One-handed operation when folded

**Disadvantages:**
- Smaller unfolded screen vs. book-style folders
- Crease more noticeable (higher stress concentration)
- Limited multitasking capabilities

**Examples:**
- Samsung Galaxy Z Flip series (1-5)
- Motorola Razr (2019, 2020, 2022, 2023, 2024, 40/40 Ultra)
- OPPO Find N2 Flip
- Huawei P50 Pocket

**Design Challenges:**

Aspect Ratio Management:
- Typical: 21:9 or 22:9 (tall and narrow)
- UI/UX challenges for app compatibility
- Flex mode: Half-folded operation (90-120° angle)

Cover Display Integration:
- Small (1-2 inch): Notifications, quick info
- Large (3-4 inch): Full app functionality
- Trade-off: larger cover display = less need for unfolding

### 3.3 Ultra-Thin Glass (UTG) Technology

Ultra-thin glass revolutionized foldable displays by offering superior scratch resistance and a more premium feel compared to polymer alternatives.

#### UTG Manufacturing Process

**Step 1: Glass Forming**
- Float glass process creates continuous ribbon
- Composition: Alkali-free aluminosilicate (similar to Gorilla Glass)
- Initial thickness: 0.5-1mm

**Step 2: Chemical Strengthening**
- Ion exchange in molten KNO₃ salt bath (400-450°C, 8-16 hours)
- Potassium ions (K⁺) replace smaller sodium ions (Na⁺) in surface layer
- Creates compressive stress layer (50-100 μm deep, 600-800 MPa)
- Significantly increases crack resistance

**Step 3: Precision Thinning**
- Chemical etching in HF-based solutions
- Controlled removal to achieve 25-50 μm thickness
- Maintains uniform thickness (±2 μm tolerance)
- Both surfaces polished simultaneously for symmetry

**Step 4: Edge Treatment**
- Laser cutting or mechanical scribing + breaking
- Edge polishing to prevent micro-crack initiation
- Rounded edges (radius >10 μm) to reduce stress concentration

**Step 5: Protective Coating**
- Anti-fingerprint oleophobic coating
- Hard coating layer (optional, for out-folding)
- Adhesive promotion layer for lamination

#### UTG vs. Polymer Substrates

| Property | UTG (50 μm) | Colorless PI | Standard PI |
|----------|-------------|--------------|-------------|
| **Thickness** | 25-50 μm | 30-60 μm | 50-100 μm |
| **Hardness** | 6-7 Mohs | 2-3 Mohs | 2-3 Mohs |
| **Scratch Resistance** | Excellent | Poor | Poor |
| **Minimum Bend Radius** | 1.5-3 mm | 1-2 mm | 1-2 mm |
| **Optical Clarity** | Excellent | Excellent | Good (yellowish) |
| **Surface Roughness** | <1 nm Ra | 2-5 nm Ra | 5-10 nm Ra |
| **Thermal Expansion** | 3 ppm/°C | 15-40 ppm/°C | 30-50 ppm/°C |
| **Cost (per unit)** | $15-25 | $5-10 | $2-5 |
| **Risk of Breakage** | Moderate | Very Low | Very Low |
| **Manufacturing Yield** | 70-85% | 90-95% | 95-98% |

**Key Insight:** UTG provides superior user experience (scratch resistance, premium feel) but requires more sophisticated handling and has higher failure risk during manufacturing.

#### UTG Failure Modes and Prevention

**Failure Mode 1: Edge Crack Initiation**
- Cause: Micro-chips on edges from handling or cutting
- Prevention: Precision edge polishing, protective frame overhang
- Detection: Automated optical inspection for edge quality

**Failure Mode 2: Particle-Induced Fracture**
- Cause: Hard particles (dust, silica) trapped during lamination create stress points
- Prevention: Class 10,000 or better cleanroom environment, adhesive filtering
- Detection: Pre-lamination inspection with high-intensity backlighting

**Failure Mode 3: Thermal Shock**
- Cause: Rapid temperature changes during processing or use
- Prevention: Gradual heating/cooling ramps, thermal matching of layers
- Detection: Thermal imaging during assembly

**Failure Mode 4: Repetitive Stress Fatigue**
- Cause: Fold cycles cause micro-crack propagation
- Prevention: Design for stress <50% of ultimate strength, crack-arrest layers
- Detection: Endurance testing to >200,000 cycles

#### Advanced UTG Technologies

**Hybrid Glass-Polymer Structures:**
- UTG for outer surface (scratch protection)
- Polymer backing layer (crack arrest)
- Achieves benefits of both materials
- Used in latest Samsung Z Fold/Flip series

**Self-Healing Coatings:**
- Polymer top layer that flows to fill micro-scratches
- Heat-activated or spontaneous healing
- Maintains smooth feel over device lifetime

**Gradient Composition Glass:**
- Varying composition through thickness
- Higher strength at surfaces, more flexible in core
- Prototype stage (MIT, Corning research)

### 3.4 Crease Management

The visible crease at the fold line remains one of the most significant challenges in foldable displays.

#### Physics of Crease Formation

**Mechanism:**
1. Repeated bending creates cyclic stress
2. Plastic deformation accumulates in polymer layers
3. Adhesive layers creep and create voids
4. Surface buckles or forms permanent deformation

**Crease Depth Evolution:**
- After 10,000 cycles: 0.05-0.1 mm
- After 50,000 cycles: 0.2-0.3 mm
- After 100,000 cycles: 0.3-0.5 mm
- After 200,000 cycles: 0.5-0.8 mm (acceptable limit)

**Factors Affecting Crease:**
- Bend radius: Smaller radius = deeper crease (exponential relationship)
- Temperature: Higher temperature accelerates creep
- Humidity: Can plasticize polymers, increasing deformation
- Folding speed: Faster folding = higher instantaneous stress

#### Crease Reduction Strategies

**Strategy 1: Optimized Bend Radius**

Relationship: Crease depth ∝ 1/r²

Increasing radius from 2mm to 3mm reduces crease depth by ~50%

Trade-offs:
- Larger radius = thicker device when folded
- Larger hinge mechanism
- Lower stress but potentially visible gap

Optimal radius for in-folding: 2.5-3mm (WIA-SEMI-011 standard)

**Strategy 2: Neutral Plane Engineering**

Goal: Position flexible layers near neutral plane (zero stress)

Approach:
```
Total structure: [Substrate / Barrier / TFT / Planarization / OLED / Encapsulation / Touch / Cover]

Neutral plane at ~40% through total thickness from bottom
Position UTG substrate asymmetrically
Thinner layers above OLED, thicker below
```

Result: 30-40% reduction in maximum strain

**Strategy 3: Stress-Relief Structures**

**Micro-Slit Patterns:**
- Laser-cut slits in rigid layers perpendicular to fold axis
- Allows localized strain relief
- Spacing: 50-100 μm
- Depth: Through brittle layers (ITO, SiN) only
- Disadvantage: Can affect optical uniformity if too large

**Accordion Structures:**
- Pre-folded polymer layers that extend/compress during bending
- Similar to expandable bellows
- Implementation: Embossed or molded patterns in substrate
- Advantage: Significantly reduces stress
- Disadvantage: Increases thickness, complex manufacturing

**Island-Bridge Architecture:**
- Rigid "islands" for active components
- Flexible "bridges" in non-active areas
- Used in some stretchable displays, less common in foldables
- Challenge: Visible seams between islands

**Strategy 4: Advanced Materials**

**Low-Modulus Adhesives:**
- Reduces stress transfer between layers
- Silicone-based adhesives (E = 1-10 MPa)
- vs. Standard acrylics (E = 1-10 GPa)
- Trade-off: Lower adhesion strength, potential delamination

**Self-Leveling Polymers:**
- Viscous polymers that flow under stress
- Can partially "heal" creases during rest periods
- Temperature-activated or photopolymerized after forming

**Gradient Modulus Structures:**
- Varying stiffness through thickness
- Stiff at surfaces (protection, support)
- Compliant in middle (strain relief)
- Manufacturing challenge: Multi-step deposition

**Strategy 5: Hinge Design Optimization**

See Section 3.5 for detailed hinge mechanisms

Key principles:
- Maintain constant bend radius throughout fold cycle
- Eliminate relative sliding between layers
- Distribute stress evenly across fold zone width

### 3.5 Hinge Mechanisms

The hinge is arguably the most critical component in a foldable device, directly impacting durability, crease visibility, and user experience.

#### Hinge Design Requirements

**Mechanical Requirements:**
- Support 200,000+ open/close cycles
- Maintain fold angle stability (not flop open/closed)
- Smooth operation (no catching or grinding)
- Precise alignment (prevent layer shearing)
- Minimize thickness and weight

**Functional Requirements:**
- Controlled bend radius (prevent over-bending)
- "Flex mode" stability at intermediate angles (90-120°)
- Dust protection (minimize ingress)
- Accommodate display thickness variation (manufacturing tolerances)

**User Experience Requirements:**
- Satisfying tactile feedback ("click" or smooth resistance)
- Quiet operation (no squeaks or rattles)
- Easy one-handed opening (for clamshells)
- Secure closed state (magnetic latch or friction)

#### Multi-Cam Hinge (Samsung Design)

**Implementation:** Galaxy Z Fold 2, 3, 4, 5 and Z Flip 3, 4, 5

**Mechanism:**
- Multiple cam assemblies on each side (typically 3-5 cams per hinge)
- Cams rotate on precision bearings
- Synchronization gear ensures all cams move together
- Spring-loaded mechanism provides adjustable resistance

**Components:**
- **Cam plates**: Hardened steel, shaped profile controls motion
- **Bearings**: Ceramic or hardened steel, for smooth rotation
- **Synchronization shaft**: Ensures left and right sides move together
- **Springs**: Torsion or compression, provide resistance and position stability
- **Housing**: Aluminum or magnesium alloy, structural support
- **Sweep brush**: Nylon fibers to sweep dust out of mechanism

**Advantages:**
- Excellent flex mode stability (can hold at any angle 75-115°)
- Smooth, controlled motion
- Compact when fully folded
- Proven durability (250,000+ cycle testing)

**Disadvantages:**
- Complex manufacturing (tight tolerances, multi-part assembly)
- Higher cost ($30-50 per hinge assembly)
- Weight penalty (15-25 grams)
- Visible gap when closed (teardrop-shaped space)

**Evolution:**
- Z Fold 1: Large gap, basic cam design
- Z Fold 2: Improved sweep mechanism, smaller gap
- Z Fold 3: Armor frame, reduced gap by 15%
- Z Fold 4: Slimmer hinge, 3mm thinner device
- Z Fold 5: Integrated hinge, waterdrop-inspired cam profile

#### Waterdrop Hinge (Huawei Design)

**Implementation:** Mate X2, Xs 2, P50 Pocket

**Mechanism:**
- Hinge creates teardrop-shaped cavity when fully folded
- Display wraps around teardrop, not folded to sharp angle
- Multiple interlocking parts (100+ components in Mate X2)

**Key Features:**
- **Larger effective radius**: 3-4mm vs. 2-2.5mm for standard hinges
- **Reduced crease**: 40-50% less visible vs. sharp fold
- **Zero gap when closed**: Sides come together completely
- **Distributed stress**: More gradual bending transition

**Advantages:**
- Minimal crease (best-in-class as of 2024-2025)
- No gap when closed (premium appearance)
- Lower stress on display (longer lifetime)
- Smoother display surface

**Disadvantages:**
- Thicker device overall (hinge protrudes)
- More complex manufacturing (200+ parts in full assembly)
- Higher cost ($40-60 per hinge)
- Larger teardrop cavity = more wasted space
- Limited flex mode capability (less stable at intermediate angles)

**Technical Details:**

Waterdrop Profile:
- Major axis: 8-10mm
- Minor axis: 3-4mm
- Elliptical approximation for cam design

Material Selection:
- Zirconium alloy for high-wear components
- Carbon fiber reinforced polymer for lightweight parts
- Liquid metal for smooth surfaces

#### Teardrop Hinge (Motorola Design)

**Implementation:** Razr (2019, 2020, 2022+)

**Mechanism:**
- Similar to waterdrop but optimized for vertical (clamshell) folding
- Display plate system with multiple hinged segments
- Creates smooth curve instead of sharp fold

**Unique Features:**
- **Zero-gap design**: When closed, no visible gap
- **Plate system**: Display attaches to rigid plates that rotate relative to each other
- **Support structure**: Prevents display from collapsing into gap

**Advantages:**
- Excellent crease management for clamshell form factor
- Compact when folded
- Premium closing action with satisfying "snap"

**Disadvantages:**
- More complex display attachment (plates add weight)
- Potential for display delamination if plates misalign
- Less flex mode stability than Samsung multi-cam

#### 360° Dual-Axis Hinges

**Concept:** Allow folding in both directions and/or multiple fold points

**Implementations:**
- **Microsoft Surface Duo**: Two separate displays with 360° hinge
- **TCL Fold 'n Roll**: Combination folding and rolling (concept)
- **LG Rollable + Fold**: Prototype combining technologies

**Challenges:**
- Significantly more complex than single-axis hinges
- Weight and thickness penalties
- Durability concerns (more moving parts = more failure points)
- Cost (>$100 for advanced dual-axis mechanisms)

**Applications:**
- Laptop-style devices (Surface Duo, Asus Zenbook Fold)
- Future mixed-mode devices (fold + roll)
- Specialized professional equipment

### 3.6 Display Stack Design for Foldables

Optimizing the layer structure is crucial for balancing performance, durability, and manufacturability.

#### Typical Stack Structure (Samsung Galaxy Z Fold 5)

**From Bottom to Top:**

1. **UTG Substrate** (30 μm)
   - Alkali-free glass, chemically strengthened
   - Custom composition for bend radius 2.5mm

2. **Barrier Layer** (2 μm)
   - 5-layer inorganic/organic/inorganic/organic/inorganic
   - WVTR <10⁻⁶ g/m²/day

3. **Backplane** (15 μm total)
   - LTPS (Low-Temperature Polysilicon) TFT array
   - 7T1C pixel circuit for compensation
   - Mobility: 80-120 cm²/V·s

4. **Planarization** (2 μm)
   - Polyimide-based, fills step height from TFTs
   - Surface roughness <5nm

5. **OLED Stack** (0.3 μm)
   - Tandem OLED structure (two emissive units)
   - RGB side-by-side via FMM (Fine Metal Mask)
   - Peak brightness: 1,750 nits

6. **Thin-Film Encapsulation (TFE)** (3 μm)
   - 7-layer barrier structure
   - Protects OLED from moisture and oxygen

7. **Touch Sensor** (1 μm)
   - On-cell capacitive touch
   - Metal mesh pattern (transparent regions)
   - Single-layer mutual capacitance

8. **Polarizer** (Optional, 10-50 μm)
   - Reduces ambient light reflection
   - Some designs omit to reduce thickness
   - Trade-off: better sunlight readability vs. thinner display

9. **Cover Film** (20-50 μm)
   - PET or TPU (thermoplastic polyurethane)
   - Provides smooth touch surface
   - Replaceable in some designs

**Total Thickness:** 50-80 μm (excluding cover film)

#### Layer-by-Layer Optimization

**Challenge 1: Balancing Rigidity and Flexibility**

Solution: Asymmetric structure
- Stiffer below neutral plane (TFT, substrate)
- More compliant above neutral plane (encapsulation, touch)
- Ratio: 60% thickness below neutral plane, 40% above

**Challenge 2: Thermal Budget**

UTG can withstand 400-500°C, enabling high-performance LTPS TFT
Polyimide typically limited to 350-400°C
Processing sequence:
1. High-temperature steps on substrate (TFT, barrier)
2. Moderate-temperature steps (OLED, 200-300°C)
3. Low-temperature steps (encapsulation, touch, <150°C)

**Challenge 3: Stress Management**

Critical layers:
- ITO (Indium Tin Oxide): Brittle, cracks at >0.5% strain
  - Solution: Silver nanowire or metal mesh alternatives
- Silicon Nitride (SiNx): Barrier layer, brittle
  - Solution: Thinner layers (<100nm), hybrid with SiOx

### 3.7 Market Analysis: Leading Foldable Devices

#### Samsung Galaxy Z Fold Series

**Z Fold 3 (2021):**
- Display: 7.6" main (2208×1768), 6.2" cover
- UTG with improved hardness coating
- Under-display camera (4MP, 400 PPI)
- IPX8 water resistance (first foldable)
- S Pen support (with special fold-compatible tip)
- Price: $1,799 USD at launch

**Z Fold 4 (2022):**
- Display: 7.6" main, 6.2" cover (slightly wider)
- Thinner hinge (-3mm), lighter (-8g)
- Improved crease (20% less visible)
- Better cameras (50MP main)
- Price: $1,799 USD

**Z Fold 5 (2023):**
- Display: 7.6" main, 6.2" cover
- Waterdrop-inspired hinge (gap nearly eliminated)
- Snapdragon 8 Gen 2 for Galaxy
- Brighter display (1,750 nits peak)
- Price: $1,799 USD

**Market Position:**
- Dominant player (70%+ foldable market share as of 2023)
- Premium positioning, high profit margins
- Ecosystem integration (Samsung DeX, S Pen, continuity)

#### Samsung Galaxy Z Flip Series

**Z Flip 3 (2021):**
- Display: 6.7" main (2640×1080, 22:9), 1.9" cover
- More affordable foldable ($999 launch price)
- Fashion-focused marketing (colors, collaborations)
- Mainstream breakthrough (millions of units)

**Z Flip 4 (2022):**
- Larger battery (3,700 mAh)
- Improved hinge durability
- Better cameras
- Price: $999 USD

**Z Flip 5 (2023):**
- Large cover display (3.4")
- Widgets and app support on cover screen
- Reduced crease visibility
- Price: $999 USD

**Market Position:**
- Best-selling foldable globally
- Appeals to style-conscious consumers
- Lower price drives adoption

#### Huawei Mate X Series

**Mate X2 (2021):**
- In-folding design (shift from original out-folding)
- 8" main display, 6.45" cover
- Waterdrop hinge (minimal crease)
- Kirin 9000 chipset
- Limited availability (no Google services)

**Mate Xs 2 (2022):**
- Out-folding design
- Ultra-lightweight (255g)
- Composite fiber material
- Improved hinge durability

**Market Position:**
- Strong in China (40%+ share domestically)
- Limited global presence due to US sanctions
- Premium pricing (¥12,999-16,999 CNY / ~$1,800-2,400 USD)

#### Motorola Razr Series

**Razr (2019):**
- First modern clamshell foldable
- Nostalgic design, innovative engineering
- Small cover display (2.7")
- Issues: Durability concerns, mid-range specs, $1,499 price

**Razr 2022 / 2023:**
- Larger cover display (2.7")
- Improved specs (Snapdragon 8+ Gen 1)
- Better battery life
- More competitive pricing ($999)

**Razr 40 / 40 Ultra (2023-2024):**
- Very large cover display (3.6" for Ultra)
- Full app functionality on cover screen
- Competitive with Samsung Z Flip
- Price: $699 (Razr 40), $999 (Razr 40 Ultra)

**Market Position:**
- Third place globally in foldables
- Gaining share in clamshell segment
- Focus on value and design

### 3.8 Manufacturing Challenges and Solutions

#### Yield Optimization

**Typical Yield Progression:**
- Early production (Gen 1): 40-60% yield
- Mature production (Gen 3+): 75-85% yield
- Target for profitability: >80% yield

**Major Defect Types:**
1. UTG cracks during lamination (25-30% of defects)
2. Particle contamination (20-25%)
3. OLED non-uniformity (15-20%)
4. Touch sensor opens/shorts (10-15%)
5. Hinge misalignment (5-10%)
6. Other (10-15%)

**Solutions:**
- Automated optical inspection at every step
- Class 1000 or better cleanrooms
- Controlled lamination pressure and temperature profiles
- Statistical process control and real-time feedback
- Sacrificial test units for process development

#### Cost Reduction Strategies

**Component Costs (Galaxy Z Fold 5, estimated):**
- Displays (main + cover): $150-180
- UTG substrate: $20-30
- Hinge mechanism: $40-50
- Other components: $200-250
- Assembly: $50-75
- **Total BOM**: ~$500-600 (vs. $1,799 retail)

**Cost Reduction Approaches:**
1. **Scale**: Higher volumes → better pricing from suppliers
2. **Integration**: Combine functions (on-cell touch, integrated hinge)
3. **Substrate shift**: Migrate from UTG to advanced polymers for mid-range models
4. **Simplified hinge**: Fewer parts, easier assembly
5. **Larger panel size**: More displays per mother glass
6. **Automation**: Reduce labor in assembly

**Price Evolution:**
- 2019: $2,000+ for flagship foldables
- 2021: $1,800 (Z Fold 3), $1,000 (Z Flip 3)
- 2023: $1,800 (Z Fold 5), $1,000 (Z Flip 5), $700 (mid-range options)
- 2025 target: $1,500 flagship, $800 mainstream, $500 entry-level

---

### 3.9 Summary

This chapter explored foldable display technology in depth:

✅ **Fold Configurations**: In-folding, out-folding, clamshell designs and their trade-offs
✅ **UTG Technology**: Manufacturing, properties, and advantages over polymer substrates
✅ **Crease Management**: Physics of crease formation and strategies to minimize visibility
✅ **Hinge Mechanisms**: Multi-cam, waterdrop, teardrop, and dual-axis designs
✅ **Market Leaders**: Samsung Galaxy Z Fold/Flip, Huawei Mate X, Motorola Razr
✅ **Manufacturing**: Yield challenges, cost structures, and optimization strategies

In **Chapter 4**, we'll examine rollable display systems, including mechanics of expansion/retraction, motorized mechanisms, and applications from smartphones to large-area TVs.

---

**WIA-SEMI-011 | Chapter 3 | Foldable Display Technology**

© 2025 SmileStory Inc. / WIA | 弘益人間 (Benefit All Humanity)
