# Chapter 6: Defect Detection and Repair Technology

## The Defect Challenge in MicroLED Manufacturing

Even with exceptional transfer yields of 99.99%, a 65-inch 4K MicroLED display with 25 million LED chips would have 2,500 defective pixels—rendering the display commercially unacceptable. Defect detection and repair technologies are therefore not optional enhancements, but essential components of MicroLED manufacturing. This chapter explores the sophisticated inspection systems and repair methodologies that make high-yield MicroLED production possible.

## Defect Types and Classification

### Transfer-Related Defects

**Missing LED (Dead Pixel)**:
- LED chip failed to transfer from donor wafer
- Most common defect type
- Frequency: 10-1,000 per million depending on transfer method
- Detection: Optical inspection or electrical testing

**Misplaced LED**:
- LED transferred but outside position tolerance (±2 μm typical)
- Can cause short circuits or poor electrical contact
- Frequency: 50-500 per million
- Detection: High-resolution vision inspection

**Damaged LED**:
- LED cracked or fractured during transfer
- May function initially but fail prematurely
- Frequency: 10-100 per million
- Detection: Optical inspection, electrical screening

**Tilted or Rotated LED**:
- LED chip at incorrect angle or orientation
- Affects optical emission pattern and electrical contact
- Frequency: 20-200 per million
- Detection: 3D vision systems, laser scanning

**Contamination**:
- Particles under LED chip
- Prevents proper bonding and electrical contact
- Frequency: 50-500 per million
- Detection: Electrical test, sometimes optical

### LED Chip Defects

**Low Brightness**:
- LED emits but at <80% expected intensity
- Caused by epitaxial defects, process variation
- Frequency: 500-5,000 per million (without binning)
- Detection: Electroluminescence measurement

**Wrong Color/Wavelength**:
- Wavelength outside specification (±5 nm typical tolerance)
- Caused by composition variation during growth
- Frequency: 1,000-10,000 per million (without binning)
- Detection: Spectral measurement

**Short Circuit**:
- Forward voltage <1.5V (vs. normal 2.5-3.5V)
- Caused by defects creating current path bypassing junction
- Frequency: 10-100 per million
- Detection: Electrical test

**Open Circuit**:
- Infinite resistance, no current flow
- Caused by contact failure or broken connections
- Frequency: 100-500 per million
- Detection: Electrical test

**Premature Degradation**:
- Accelerated brightness loss during burn-in
- Caused by defects, contamination, or material issues
- Frequency: 100-1,000 per million
- Detection: Burn-in testing (time-consuming)

## Inspection Technologies

### Automated Optical Inspection (AOI)

AOI represents the first line of defense for defect detection, providing high-throughput screening before electrical testing.

**System Components**:

**Imaging System**:
- High-resolution cameras: 5-20 megapixels
- Lens magnification: 2-10× depending on chip size
- Field of view: 10-100 mm
- Resolution: 1-5 μm per pixel
- Multiple cameras for large area coverage

**Illumination**:
- LED ring lights or structured illumination
- Multiple wavelengths (white, red, green, blue, UV, IR)
- Dark field and bright field modes
- Polarized light options

**Positioning System**:
- XY stage with <1 μm repeatability
- Travel range: 300-1,000 mm
- Speed: 100-500 mm/s
- Vibration isolation critical

**Defect Detection Algorithms**:
- Pattern recognition (missing chips)
- Edge detection (position verification)
- Intensity analysis (damaged chips)
- Particle detection (contamination)

**Performance Metrics**:
- Inspection speed: 50-200 wafers or panels per hour
- Detection rate: >99% for missing/damaged chips
- False positive rate: <0.1% (1,000 per million)
- Minimum detectable defect: 5-10 μm

**Limitations**:
- Cannot detect electrical defects
- Difficulty with transparent or reflective surfaces
- Wavelength/brightness variations not detected
- Requires good contrast and lighting

### Electroluminescence (EL) Testing

EL testing drives each LED and measures its light output, providing comprehensive functional verification.

**Test Sequence**:

1. **Contact**: Probe array contacts TFT backplane
2. **Power**: Apply voltage to specific LED or group
3. **Measurement**: Camera captures emitted light
4. **Analysis**: Measure intensity, color, position
5. **Index**: Move to next group
6. **Repeat**: Test all LEDs

**Equipment**:

**Probe Card/Contactor**:
- Spring-loaded pins contact display electrodes
- Pin count: 100-10,000 depending on test strategy
- Contact force: 5-50 grams per pin
- Lifetime: 100,000-1,000,000 contacts

**Current Source**:
- Precision current: 1 μA to 10 mA per channel
- Channels: 100-10,000 parallel
- Accuracy: ±1% of setting
- Voltage compliance: 0-5V

**Imaging System**:
- High-sensitivity camera (low-noise CMOS or EMCCD)
- Dynamic range: 16-bit (65,536 levels)
- Captures multiple LEDs simultaneously
- Integration time: 10-1000 milliseconds

**Spectrometer** (optional):
- Measures wavelength of each LED
- Spectral range: 380-780 nm
- Resolution: 1-5 nm
- Fiber optic probes or imaging spectrometer

**Test Modes**:

**Pixel-by-Pixel**:
- Test each LED individually
- Highest accuracy
- Slowest: 1-10 million pixels = 1-10 hours
- Used for high-end displays

**Block Testing**:
- Test multiple LEDs simultaneously (e.g., 100×100 block)
- Faster but less precise
- Identifies defective regions for detailed testing
- Used for screening

**One-Time Testing**:
- Entire display illuminated
- Camera captures full image
- Identifies gross defects only
- Fast: seconds per panel
- Limited accuracy for individual pixel characterization

**Performance**:
- Detects: Dead pixels, low brightness, wavelength errors, shorts
- Accuracy: ±5% brightness, ±2 nm wavelength
- Throughput: 10-100 panels per hour (depends on resolution and test mode)
- Cost: $500,000-$5,000,000 per system

### Advanced Inspection Methods

**3D Surface Profiling**:
- Laser triangulation or confocal microscopy
- Measures LED height and tilt
- Accuracy: ±0.1 μm in Z-axis
- Throughput: Limited, used for sampling

**Thermography**:
- Infrared camera during LED operation
- Detects hot spots indicating defects or poor thermal contact
- Can identify shorted LEDs or high-resistance connections
- Throughput: Fast (seconds per panel)

**Lock-In Thermography**:
- Modulated current + synchronized IR imaging
- Enhanced sensitivity to subtle defects
- Research tool, not yet production-ready

**Photocurrent Mapping**:
- Laser scanning induces photocurrent in LEDs
- Maps spatial response
- Can detect defects not visible in forward bias
- Research stage

## Defect Repair Technologies

### Laser Ablation Repair

**Principle**: Focused laser removes defective LED chip, enabling replacement or redundant pixel activation

**Process**:

1. **Identify Defect**: From inspection data
2. **Position**: Move laser to defective LED location
3. **Ablate**: Pulsed laser vaporizes LED and contact materials
4. **Verify**: Inspect removal completeness
5. **Clean**: Remove debris
6. **Replace/Activate**: Install new LED or activate redundant pixel

**Laser Parameters**:
- Wavelength: 355 nm (UV) or 532 nm (green)
- Pulse duration: 10-100 nanoseconds
- Energy: 10-500 microjoules per pulse
- Spot size: 5-30 micrometers
- Repetition rate: 1-100 kHz

**Advantages**:
- Non-contact process
- High precision (±1 μm)
- Fast: 0.5-5 seconds per LED
- No damage to adjacent pixels

**Challenges**:
- Debris generation requires cleanup
- Potential damage to substrate
- Incomplete removal can cause electrical shorts
- Expensive equipment ($200,000-$1,000,000)

**Success Rate**: 90-98% successful removal

### Redundant Pixel Activation

**Design Philosophy**: Incorporate extra LEDs in pixel structure; activate redundant LED when primary fails

**Architectures**:

**Dual-LED Pixels**:
- Each subpixel has two LED chips
- Electrical routing allows independent addressing
- Use better performing LED or average both
- If one fails, use the other
- Area penalty: 2× LED chips (but same visible pixel count)

**Clustered Redundancy**:
- Extra LEDs distributed across display region
- When nearby LED fails, activate redundant LED
- Requires flexible routing and driver capability
- Lower area penalty than dual-LED

**Spare Rows/Columns**:
- Entire extra rows or columns of LEDs
- Activated to replace failed row/column
- Simpler routing but coarse granularity
- Common in OLED, less suitable for MicroLED

**Implementation**:

**TFT Design**:
- Additional thin-film transistors for redundant paths
- Multiplexers to select active LED
- Fuses or anti-fuses to permanently configure
- Increased TFT complexity and cost

**Driver IC Programming**:
- Defect map stored in memory
- Driver re-maps image data to active pixels
- Can compensate for brightness variation
- Requires non-volatile memory

**Advantages**:
- No physical repair needed
- Instantaneous repair (electronic activation)
- Can be performed post-assembly
- Enables >99.9% final yield from >99% transfer yield

**Trade-offs**:
- 20-100% increase in LED chip count
- More complex TFT backplane
- Higher manufacturing cost
- Increased power consumption (if both LEDs active)

**Effectiveness**:
- Single redundancy: 99.0% → 99.99% yield improvement
- Dual redundancy: 99.0% → 99.9999% yield improvement

### Individual LED Replacement

**Concept**: Remove defective LED and install new chip in same location

**Process**:

1. **Remove**: Laser ablation or mechanical removal of defective LED
2. **Clean**: Remove residual materials and debris
3. **Prepare**: Apply fresh adhesive or solder to pad
4. **Place**: Position new LED chip (from KGD - known good die stock)
5. **Bond**: Cure adhesive or reflow solder
6. **Test**: Verify repair success

**Equipment**:
- Laser removal system
- Pick-and-place tool with <1 μm accuracy
- Vision alignment system
- Bonding tool (thermal, UV, ultrasonic)

**Challenges**:
- Time-consuming: 10-60 seconds per LED
- Low throughput: 100-500 repairs per hour
- Cost: $0.10-$1.00 per repair
- Risk of damaging adjacent pixels
- Difficulty maintaining cleanliness

**When Used**:
- Small displays with low defect counts
- Ultra-premium displays justifying labor cost
- Prototype and development

**Success Rate**: 80-95% successful repair

### Circuit Redundancy and Bypass

**Concept**: Electrical routing bypasses defective pixel without replacement

**Series String Configuration**:
- Multiple LEDs in series (power delivery efficiency)
- Parallel bypass paths with higher resistance
- Defective LED bypassed automatically or via fuse
- Current redistributes to functioning LEDs

**Implementation**:
- Bypass diodes or resistors fabricated into TFT
- Laser-activated or electrically-activated bypass
- Minimal impact on display appearance if <1% pixels bypassed

**Limitations**:
- Only works for certain defect types (open circuits)
- Shorted LEDs cannot be bypassed without isolation
- Brightness uniformity affected if many bypasses in region
- Requires design foresight

### Pixel Compensation Algorithms

**Concept**: Software compensation for defective or sub-optimal pixels

**Brightness Calibration**:
- Measure actual brightness of each LED
- Store correction factors in driver IC memory
- Adjust drive current to normalize brightness
- Can compensate for ±20-30% variations

**Color Calibration**:
- Measure wavelength of each LED
- Adjust RGB mix to achieve target white point
- Requires spectral measurement during production
- Storage: 8-16 bits per subpixel

**Defect Masking**:
- Identify defective pixels
- Render image around defects (edge smoothing, etc.)
- Effective for small defect counts (<0.1%)
- Not acceptable for premium displays

**Implementation**:
- Non-volatile memory in driver ICs
- Calibration performed during manufacturing
- Can update in field for displays with persistent storage

**Effectiveness**:
- Brightness uniformity: <5% variation after calibration
- Color uniformity: ΔE <2 after calibration
- Cannot repair truly dead pixels
- Extends acceptable yield range

## Defect Budgets and Yield Modeling

### Defect Budget Allocation

For a target final yield of 99.995% (5 defects per million subpixels):

**Budget Breakdown Example**:
- Transfer defects: 300 ppm (99.97% yield)
- LED chip defects: 100 ppm (99.99% yield)
- Bonding defects: 50 ppm (99.995% yield)
- TFT backplane defects: 50 ppm (99.995% yield)

**Total**: 500 ppm (99.95% yield) before repair

**After Repair** (95% repair success on 500 ppm):
- Remaining defects: 25 ppm
- **Final Yield**: 99.9975%

### Yield Modeling

**Compound Yield**:
Yfinal = Ytransfer × YLED × Ybonding × YTFT × (1 + Prepair × Yrepair)

Where:
- Y = yield (fraction of good units)
- Prepair = fraction of defects that can be repaired
- Yrepair = success rate of repair process

**Example Calculation**:

Initial yields:
- Transfer: 99.90%
- LED chips: 99.95%
- Bonding: 99.98%
- TFT: 99.90%

Compound yield before repair: 99.73%

If 80% of defects repairable at 95% success:
Final yield = 99.73% × (1 + 0.0027 × 0.80 × 0.95) = 99.93%

**Sensitivity Analysis**:
- Transfer yield has highest impact (largest defect contributor)
- Repair capability critical for achieving >99.9% final yield
- TFT backplane yield often overlooked but significant

## Inspection and Repair Integration

### Production Flow

**Stage 1: Post-Transfer AOI**
- Detect missing and damaged chips
- Map defect locations
- Decision: Immediate repair or continue?

**Stage 2: Pre-Bonding EL Test**
- Functional test before encapsulation
- Detect electrical defects
- Last opportunity for easy repair

**Stage 3: Repair Station**
- Laser ablation or replacement
- Redundant pixel activation
- Verify repair success

**Stage 4: Post-Repair Verification**
- Re-test repaired pixels
- Update defect map

**Stage 5: Final Inspection**
- Full-panel EL test
- Brightness and color calibration
- Generate calibration data

**Stage 6: Burn-In** (for premium displays)
- Operate at elevated temperature (60-85°C)
- High current stress
- Duration: 24-168 hours
- Accelerates infant mortality failures

**Stage 7: Final QC**
- Visual inspection
- Functional testing
- Cosmetic assessment

### Throughput and Cost Impact

**Inspection Time**:
- AOI: 2-10 minutes per panel
- EL test: 10-60 minutes per panel
- 3D profiling (if used): 5-20 minutes per panel

**Repair Time**:
- Laser ablation: 0.5-5 seconds per defect
- LED replacement: 10-60 seconds per defect
- Redundancy activation: <1 second per defect (electronic)

**For 65" 4K Panel**:
- Expected defects at 99.9% transfer yield: 25,000
- Laser ablation repair: 25,000 × 2 sec = 13.9 hours
- LED replacement repair: 25,000 × 30 sec = 208 hours (not practical)
- Redundancy activation: 25,000 × 0.1 sec = 42 minutes

**Conclusion**: Redundancy activation is only practical repair method for high-resolution displays

**Cost Allocation** (typical 65" display):
- AOI equipment amortization: $50-100 per panel
- EL test equipment amortization: $100-300 per panel
- Repair (redundancy): $50-200 per panel (chip cost)
- Repair (laser + replacement): $1,000-5,000 per panel (not practical)
- Calibration data storage: <$10 per panel

**Total inspection and repair**: $200-600 per panel (with redundancy design)

## Quality Standards and Specifications

### ISO 9241-305 (Pixel Defect Standards)

**Class I** (Premium):
- Zero dead pixels allowed
- Zero bright pixels allowed
- Target for professional and premium consumer displays

**Class II** (Standard):
- ≤2 dead pixels per million
- ≤2 bright pixels per million
- ≤5 total defects per million
- Typical consumer displays

**Class III** (Commercial):
- ≤5 dead pixels per million
- ≤15 bright pixels per million

**Class IV** (Industrial):
- ≤50 dead pixels per million
- ≤150 bright pixels per million

### MicroLED-Specific Metrics

**Cluster Defects**:
- No more than 2 defective pixels within 15 mm radius
- Prevents visually distracting defect patterns

**Uniformity After Calibration**:
- Brightness uniformity: <5% variation
- Color uniformity: ΔE <2
- Measured at 5×5 or 9×9 point grid

**Reliability**:
- <0.01% additional pixel failures per 10,000 hours
- Demonstrates stability of repair and manufacturing

## Future Trends

### AI-Powered Defect Detection

**Machine Learning Applications**:
- Pattern recognition for subtle defects
- Prediction of latent defects (may fail later)
- Optimization of inspection parameters
- Classification of defect root causes

**Benefits**:
- Higher detection rates
- Lower false positives
- Faster inspection
- Automated root cause analysis

**Status**: Early deployment in leading manufacturers

### In-Situ Repair

**Concept**: Repair occurs during transfer process

**Self-Healing Approaches**:
- Automatic retry of failed transfers
- Real-time defect detection during transfer
- Immediate replacement from spare chip array

**Status**: Research phase, promising for throughput improvement

### Wafer-Level Burn-In and Repair

**Concept**: Test and repair at wafer scale before transfer

**Advantages**:
- Easier access to chips before transfer
- Higher throughput
- Prevents transfer of known-bad chips

**Challenges**:
- Requires wafer-level electrical contact
- Repair limited to chip replacement (entire wafer scrap if unfixable)
- Not compatible with all transfer methods

### Predictive Maintenance

**Approach**: Monitor process parameters to predict defect formation

**Data Sources**:
- Transfer tool telemetry
- AOI trends over time
- Environmental conditions
- Material lot tracking

**AI Analysis**:
- Identifies correlations between process variation and defects
- Predicts equipment maintenance needs
- Suggests process adjustments

## Conclusion

Defect detection and repair are critical enablers of commercial MicroLED displays. The combination of high-precision inspection, strategic redundancy design, and targeted repair enables achievement of >99.99% final yields despite the challenges of handling millions of microscopic components.

Key success factors:
- Multi-stage inspection catching different defect types
- Redundant pixel architecture as primary repair method
- Laser ablation for targeted fixes
- Comprehensive calibration for uniformity
- Statistical process control for continuous improvement

The WIA-SEMI-010 standard defines defect classification, inspection requirements, repair methodologies, and quality metrics to ensure consistent industry practices and enable inter-vendor compatibility.

---

**Defect and Repair Metrics:**
- Target final yield: >99.995% (Class I premium displays)
- AOI detection rate: >99% for physical defects
- EL test coverage: 100% of pixels
- Redundancy repair success: >99%
- Laser ablation success: 90-98%
- Post-calibration uniformity: <5% brightness, ΔE<2 color

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
