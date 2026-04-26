# Chapter 4: Rollable Display Systems

## Expandable Screens: From Compact to Expansive

---

### 4.1 Introduction to Rollable Displays

Rollable displays represent the next frontier in flexible screen technology, offering dynamic screen size adjustment through mechanical extension and retraction. Unlike foldable displays that transition between two discrete states, rollable displays can provide continuously variable screen area.

#### Key Advantages of Rollable Design

**Continuous Size Adjustment:**
- Expand from phone (6-7") to tablet (8-10") smoothly
- User can select exact size needed for current task
- No discrete "folded" vs. "unfolded" states

**No Visible Crease:**
- Display rolls around constant radius cylinder
- No repeated folding at same location
- More uniform stress distribution

**Compact Form Factor:**
- Rolls into small cylinder when retracted
- Can be thinner than foldable when fully extended
- Efficient use of internal volume

**Unique Capabilities:**
- Partial extension for specific UI layouts
- Animated transitions between sizes
- Potential for very large extension ratios (4:1 or more)

### 4.2 Rollable Mechanisms

#### Motorized Roll-Out Systems

**LG Rollable (Smartphone, Prototype 2021):**

Mechanism:
- Motor-driven roller at one end
- Display wraps around roller (radius 5mm)
- Rails guide display during extension/retraction
- Compact: 6.8" screen
- Extended: 7.4" screen (0.6" expansion)

Components:
- **Brushless DC motor**: 5,000 RPM max, gear-reduced to 20-50 RPM at roller
- **Roller shaft**: Aluminum, 5mm diameter, precision bearings
- **Guide rails**: Left and right tracks prevent buckling
- **Position sensors**: Hall effect or optical encoder for precise control
- **Spring-loaded tensioner**: Maintains slight tension to prevent slack

**Technical Specifications:**
- Extension time: 0.8-1.5 seconds
- Retraction time: 1.0-2.0 seconds
- Power consumption: 1-2W during motion, 0W holding
- Lifespan: 100,000+ cycles (vs. 200,000 for foldables)
- Minimum bend radius: 5mm

**Advantages:**
- Automatic, motorized operation
- Precise position control
- Smooth, premium user experience

**Disadvantages:**
- Motor adds weight (10-15g) and cost ($20-30)
- Potential for motor failure (mechanical complexity)
- Battery drain from motor operation
- Slower than manual folding (1-2 seconds vs. instant)

---

#### Manual Slide-Out Mechanisms

**Concept Devices:** OPPO X 2021, TCL Fold 'n Roll

**Mechanism:**
- User manually pulls to extend display
- Spring-loaded retraction or manual push to retract
- No motor required (cost and weight savings)

**Spring-Assisted Design:**
- Constant force spring stores energy during extension
- Gentle automatic retraction when released
- Damper prevents snap-back (injury risk)

**Ratchet Locking:**
- Multiple locked positions (e.g., 6.5", 7.0", 7.5", 8.0")
- User can select intermediate sizes
- Release button or pull further to change position

**Advantages:**
- Simpler mechanism (fewer failure points)
- No power consumption
- Lighter weight
- Lower cost
- Faster operation (instant manual extension)

**Disadvantages:**
- Requires two hands for operation
- Potential for uneven pulling (display damage)
- Mechanical locking can wear over time
- Less precise positioning

---

#### Rollable TV Mechanisms

**LG Signature OLED R (65", Consumer Product, $100,000 USD):**

**Mechanism:**
- Display rolls into base cabinet
- Precision roller at bottom
- Motor-driven lift system
- Three modes: Full View (fully extended), Line View (partially extended), Zero View (fully retracted)

**Engineering Challenges at Large Scale:**

1. **Display Weight**: 65" OLED panel = 5-8 kg
   - Requires robust motor (50-100W)
   - Support rollers every 10-15 cm to prevent sagging
   - Tensioning system to keep display taut

2. **Rollability**: Larger panels more prone to buckling
   - Minimum bend radius increases with size
   - 65" panel requires 8-10 cm diameter roller
   - Thicker panel needed for rigidity when extended

3. **Precision**: Extension must be perfectly even across width
   - Dual motors on left and right sides, synchronized
   - Position sensors every 20 cm
   - Feedback control to correct for uneven lifting

4. **Speed**: 10-15 seconds for full extension/retraction
   - Slower = more premium feeling, lower stress
   - Faster = better user experience

5. **Noise**: Acoustic dampening required
   - Motor noise <40 dB
   - Roller bearing noise minimization
   - Vibration isolation from cabinet

**Cost Breakdown (LG OLED R):**
- 65" Rollable OLED panel: $10,000-15,000
- Precision roller and motor assembly: $5,000-8,000
- Cabinet and base: $3,000-5,000
- Electronics and control: $2,000-3,000
- Sound system (included): $2,000-3,000
- Assembly and margin: $78,000+
- **Retail price**: $100,000

**Market Reality:**
- Ultra-luxury positioning
- Very limited sales (<1,000 units estimated)
- Proof of concept for technology
- Future cost reduction potential to $10,000-20,000 range

### 4.3 Display Stack Optimization for Rolling

#### Substrate Selection for Rollable

**Polyimide (PI) Preferred:**
- Can achieve <5mm bend radius reliably
- Less risk of catastrophic failure vs. UTG
- Lower cost for large-area displays
- Easier to manufacture in continuous roll form

**UTG Challenges:**
- Higher risk at 5mm radius (closer to ultimate bend limit)
- Potential for edge cracks during repeated rolling
- Cost premium not justified by user experience (no crease to improve)

**Hybrid Approach:**
- Thin PI substrate (30 μm)
- UTG reinforcement in extended (flat) region only
- Transition zone between UTG and PI-only
- Best of both: scratch resistance when extended, rollability

---

#### Stress Analysis for Continuous Curvature

**Stress Distribution:**
- Unlike folding (localized high stress), rolling distributes stress
- Longer stress zone = lower peak stress
- Curvature is constant (radius doesn't change with each cycle)

**Fatigue Advantage:**
- Same location not repeatedly folded (vs. foldables)
- Entire rolled length experiences stress
- Lower cycle count per unit area = longer lifetime

**Calculation Example:**

Rollable smartphone extending from 80mm to 120mm:
- Extended length = 120 mm
- Retracted length = 80 mm (40 mm visible, 40 mm rolled)
- Roller radius = 5 mm

Circumference of roller = 2π × 5mm = 31.4mm
40mm rolled section wraps 1.27 turns around roller

After 100,000 cycles:
- Outer edge experiences 127,000 wraps
- vs. 100,000 folds at exact same line for foldable
- Stress distributed over 40mm length vs. <1mm fold zone

**Result**: Potentially 2-3x longer lifetime for same stress level

### 4.4 Guide Rail Engineering

Preventing buckling and ensuring smooth operation requires precision guide rails.

#### Rail Design Requirements

**Alignment Tolerance:**
- Parallelism: <0.1mm over entire length
- Gap consistency: ±0.05mm
- Surface finish: <1μm Ra (smooth gliding)

**Material Selection:**
- Stainless steel: Corrosion resistant, precise machining
- Anodized aluminum: Lightweight, adequate hardness
- Polymer coating: Reduces friction, quieter operation

**Friction Coefficient:**
- Target: μ < 0.15 (low friction)
- Achieved via: DLC (Diamond-Like Carbon) coating, Teflon, or PTFE
- Balance: Too low friction = display extends unintentionally, too high = motor strain

---

#### Anti-Buckling Mechanisms

**Problem:** When partially extended, unsupported display section can buckle under own weight or external force.

**Solutions:**

1. **Tensioning System:**
   - Spring-loaded roller applies gentle tension (10-50 grams force)
   - Keeps display taut
   - Must not over-tension (causes stress and motor load)

2. **Support Rollers:**
   - Small diameter rollers (2-3mm) every 5-10 cm
   - Support display from underneath
   - Freely rotate to minimize friction

3. **Air Cushion:**
   - Gentle air pressure from below (prototype concept)
   - Provides contactless support
   - Prevents sagging without friction
   - Requires quiet air pump

4. **Rigid Backing in Extended State:**
   - Deployable backing plate extends with display
   - Rolls into slot when retracting
   - Adds complexity but provides excellent rigidity when extended

### 4.5 Rollable Display Applications

#### Expandable Smartphones

**Concept:** OPPO X 2021, Motorola Rizr (2023 concept)

**Use Cases:**
- Compact size in pocket (6.5")
- Expand for reading, video (7.5-8.5")
- One-handed to two-handed transition

**Challenges:**
- Cost ($1,500-2,000 estimated)
- Durability perception
- App compatibility (varying aspect ratios)
- Dust ingress at expansion slot

**Market Outlook:**
- Niche product for enthusiasts
- Premium positioning above foldables
- Estimated 2027-2028 for consumer availability

---

#### Portable Monitors

**Concept:** Rollable display that extends from compact tube

**Specifications:**
- Retracted: 2-3" diameter cylinder, 12-18" long
- Extended: 15-17" diagonal display
- Thickness: <5mm when unrolled
- Weight: 300-500 grams

**Applications:**
- Travel professionals
- Field work (engineering, design)
- Secondary display for laptops
- Presentations

**Technical Feasibility:**
- OLED on PI substrate: Ready today
- Portable battery: 20-30 Wh for 4-6 hours
- Wireless connectivity: Miracast, AirPlay, USB-C DisplayPort Alt Mode

**Market Potential:**
- Price target: $500-800
- Competes with foldable portable monitors
- Unique form factor advantage
- Estimated market entry: 2026

---

#### Rollable TVs

**LG OLED R** (covered in Section 4.2)

**Alternative Concepts:**

**Ceiling-Mounted Projector Screen Style:**
- TV rolls down from ceiling cavity
- Hidden when not in use (clean aesthetics)
- Manual or motorized
- Display faces forward (vs. OLED R which rolls into base)

**Vertical Roll-Out:**
- Extends upward from cabinet
- More stable (less cantilever stress)
- Easier cable management

**Dual-Sided Display:**
- Rolls from center, extends both up and down
- Creates very tall display (32:9 aspect ratio or more)
- Gaming, productivity use cases

### 4.6 Durability Testing for Rollables

#### WIA-SEMI-011 Rollable Testing Protocol

**Test Conditions:**
- Temperature: 25°C ± 2°C
- Humidity: 50% ± 5% RH
- Roll/unroll speed: 10-50 mm/second
- Rest period: 1 second between cycles

**Test Sequence:**
1. Visual inspection (baseline): Photograph extended display, measure uniformity
2. 10,000 cycles: Check for visible defects, measure brightness uniformity
3. 50,000 cycles: Detailed inspection, optical microscopy of edges
4. 100,000 cycles: Full electrical and optical testing
5. 150,000 cycles: Destructive testing (push to failure)

**Pass Criteria:**
- 100,000 cycles minimum with <5% brightness degradation
- <10% increase in non-uniformity (mura)
- No visible defects (cracks, delamination, dead pixels)
- Touch function 100% operational
- Roll/unroll function smooth (no jamming)

**Accelerated Testing:**
- Elevated temperature (40-60°C): Accelerates polymer aging
- High humidity (80% RH): Tests barrier effectiveness
- Rapid cycling (100 mm/s): Simulates abuse conditions

---

#### Common Failure Modes

**1. Edge Delamination:**
- Cause: Stress concentration at display edges during rolling
- Prevention: Rounded edge design, edge sealant
- Detection: Optical inspection, peel testing

**2. Rail Wear:**
- Cause: Friction between display and guide rails
- Prevention: Low-friction coatings, precision manufacturing
- Detection: Friction force measurement, acoustic analysis

**3. Motor Failure:**
- Cause: Overload, bearing wear, electronic failure
- Prevention: Adequate motor sizing, feedback control
- Detection: Current monitoring, vibration analysis

**4. Roller Surface Damage:**
- Cause: Repeated contact with display, particle contamination
- Prevention: Hard-anodized aluminum or ceramic coating, dust seals
- Detection: Surface profilometry, optical inspection

**5. Display Buckling:**
- Cause: Loss of tension, rail misalignment
- Prevention: Proper tensioning, precision assembly
- Detection: Visual inspection, gap measurement

### 4.7 Cost Analysis and Market Viability

#### Component Costs (Rollable Smartphone, Estimated)

- Rollable OLED display (7-8"): $120-150
- Motor and control electronics: $30-40
- Precision roller and bearings: $20-30
- Guide rails and structure: $25-35
- Other components (battery, SoC, cameras, etc.): $250-300
- Assembly: $75-100
- **Total BOM**: ~$550-700 (vs. ~$500-600 for foldable flagship)

**Retail Price Projection:**
- 2025-2026 launch: $1,800-2,200
- 2027-2028 (mature production): $1,400-1,800
- 2030+ (mainstream): $1,000-1,400

---

#### Market Positioning

**Advantages vs. Foldables:**
- No crease (better visual experience)
- Continuously variable size (more flexible use)
- Novelty factor (differentiation)

**Disadvantages vs. Foldables:**
- Higher cost (motor, rails, complexity)
- Slower operation (1-2 seconds vs. instant fold)
- Durability perception (moving parts)
- Less compact (cannot fold to half size)

**Target Market:**
- Technology enthusiasts and early adopters
- Professionals needing variable screen size
- Users prioritizing visual quality (no crease)
- Smaller volume than foldables (estimated <5% of foldable market)

---

### 4.8 Future Directions for Rollable Technology

#### Bi-Directional Rollable

**Concept:** Extends in two directions from center

Benefits:
- Symmetrical design (balanced weight)
- Larger expansion ratio (6" compact to 12" extended)
- Center portion always flat (optimal for cameras, sensors)

Challenges:
- Two motors and roller assemblies
- Synchronization between left and right
- More complex housing and sealing

---

#### Wearable Rollables

**Smart Watch Concept:**
- Standard watch face when retracted
- Extends to cover forearm when needed
- Display wraps around curved wrist

**Technical Requirements:**
- Ultra-flexible substrate (2mm bend radius for wrist curvature)
- Lightweight mechanism (<20g)
- Low power motor (<0.5W)
- Stretchable electronics for conformability

**Applications:**
- Extended notifications and messaging
- Health monitoring displays (heart rate graphs, etc.)
- Navigation (large map on forearm)

**Timeline:** Research prototypes demonstrated, consumer products 2028+

---

#### Transparent Rollable Displays

**Concept:** Rollable display that is transparent when not displaying content

**Technologies:**
- Transparent OLED (T-OLED): 40-60% transparency
- Transparent micro-LED: 70-85% transparency
- Switchable between opaque (content) and transparent (see-through)

**Applications:**
- Augmented reality windows (roll down from ceiling when needed)
- Automotive windshield HUDs (heads-up displays)
- Smart windows in buildings
- Retail displays (product behind transparent screen)

**Challenges:**
- Transparent backplane (requires oxide TFT or organic TFT, not LTPS)
- Transparent electrodes (silver nanowire or graphene, not ITO at thickness needed)
- Brightness (transparent OLEDs are less efficient)
- Cost (even higher than standard rollable)

**Market Entry:** 2028-2030 for niche applications

---

### 4.9 Summary

This chapter explored rollable display technology:

✅ **Mechanisms**: Motorized, manual, and large-scale rollable TV systems
✅ **Display Stack**: Substrate optimization, stress distribution advantages
✅ **Guide Rails**: Precision engineering for smooth, reliable operation
✅ **Applications**: Expandable smartphones, portable monitors, rollable TVs
✅ **Durability**: Testing protocols and common failure modes
✅ **Market Analysis**: Cost structure, positioning vs. foldables, target markets
✅ **Future Tech**: Bi-directional, wearable, and transparent rollables

In **Chapter 5**, we'll examine stretchable display innovation, including elastic substrates, serpentine interconnects, micro-LED arrays, and applications in wearables and conformable electronics.

---

**WIA-SEMI-011 | Chapter 4 | Rollable Display Systems**

© 2025 SmileStory Inc. / WIA | 弘益人間 (Benefit All Humanity)
