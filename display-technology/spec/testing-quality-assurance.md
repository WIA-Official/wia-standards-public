# Display Testing and Quality Assurance Specification

**WIA-SEMI-008 Display Technology Standard**
**Version**: 1.0
**Last Updated**: 2025

---

## Overview

This specification defines testing methodologies, quality metrics, and acceptance criteria for display panels across brightness, color accuracy, response time, contrast, dead pixels, HDR performance, and environmental durability.

---

## 1. Brightness Testing

### 1.1 Peak Brightness Measurement

**Equipment Required:**
- Spectroradiometer or calibrated photometer
- Warm-up: 30 minutes minimum
- Stabilization: ±2% over 5 minutes

**Test Pattern:**
- Full white screen (100% RGB or 255,255,255)

**Measurement Procedure:**
1. Position sensor perpendicular to display center
2. Distance: 50cm (±5cm) from screen
3. Record luminance in cd/m² (nits)
4. Repeat 3 times, average results

**Peak Brightness Standards:**

| Application | Minimum (nits) | Target (nits) | Excellent (nits) |
|------------|----------------|---------------|------------------|
| Budget Mobile | 300 | 400 | 500+ |
| Premium Mobile | 500 | 800 | 1,000+ |
| Monitor (SDR) | 250 | 300 | 400+ |
| HDR Monitor | 400 | 600 | 1,000+ |
| TV (SDR) | 300 | 400 | 600+ |
| TV (HDR) | 400 | 700 | 1,000+ |

**HDR Peak Measurement:**
- Window size: 10% of screen area
- Sustained vs peak differentiation
- OLED ABL characterization

### 1.2 Brightness Uniformity

**Test Pattern:**
- Solid white (100% RGB)
- Solid gray (50% RGB)

**Measurement Grid:**
- 9-point: Standard (3×3 grid)
- 13-point: Enhanced (center + 4 corners + 8 edge midpoints)
- 25-point: Professional (5×5 grid)

**Calculation:**
```
Uniformity (%) = (1 - ((Max - Min) / Max)) × 100%
```

**Acceptance Criteria:**

| Grade | Uniformity | Application |
|-------|-----------|-------------|
| Excellent | >95% | Professional, medical |
| Good | 90-95% | Premium consumer |
| Acceptable | 85-90% | Standard consumer |
| Poor | <85% | Reject or budget |

**Common Defects:**
- Edge brightening (backlight leakage)
- Center hot spot
- Clouding (backlight non-uniformity)
- Vignetting (corner darkening)

### 1.3 Sustained Brightness (OLED)

**ABL (Automatic Brightness Limiting) Testing:**
1. Measure peak (10% window)
2. Measure sustained (50% window, 100% window)
3. Document brightness reduction curves
4. Time-to-stabilization measurement

**Requirements:**
- Gradual reduction (no abrupt drops)
- Predictable behavior
- User-perceivable but not distracting

---

## 2. Color Accuracy Testing

### 2.1 Delta E (ΔE) Measurement

**Equipment:**
- Spectroradiometer (preferred) or colorimeter
- Calibrated to CIE standards

**Test Patterns:**
- 24-patch ColorChecker (or equivalent)
- Grayscale ramp (0-100% in 10% steps)
- Skin tones (critical for photography/video)

**Delta E Formula (CIEDE2000):**
Industry-standard perceptually uniform metric.

**Interpretation:**

| ΔE Value | Quality | Application |
|----------|---------|-------------|
| <0.5 | Reference | High-end professional |
| <1.0 | Excellent | Professional color work |
| <2.0 | Professional | Photography, design |
| <3.0 | Acceptable | Consumer, general use |
| >3.0 | Poor | Noticeable errors |

**Acceptance Criteria:**
- Professional displays: Average ΔE < 2, Max ΔE < 3
- Consumer displays: Average ΔE < 3, Max ΔE < 5

### 2.2 White Point Accuracy

**Standard White Points:**
- D65 (6500K): Standard video/computing
- D50 (5000K): Print production
- DCI (6300K): Cinema

**Measurement:**
- Full white screen
- Record CIE x,y chromaticity coordinates
- Calculate correlated color temperature (CCT)

**Tolerance:**
- Professional: ±200K (Delta uv < 0.002)
- Consumer: ±500K

### 2.3 Gamma Accuracy

**Target Gamma:**
- 2.2: Standard computer displays
- 2.4: Video in dark environments
- 2.6: Cinema projection
- ST.2084 (PQ): HDR content

**Measurement:**
1. Display grayscale steps (0-100%)
2. Measure luminance at each step
3. Calculate actual gamma: L = (V/255)^γ
4. Compare to target curve

**Acceptance:**
- Professional: Gamma ±0.05
- Consumer: Gamma ±0.1

### 2.4 Color Gamut Coverage

**Standard Gamuts:**
- sRGB: Web, office, general computing
- DCI-P3: Cinema, premium displays
- Adobe RGB: Photography, print
- Rec.2020: Future HDR/UHD

**Measurement:**
1. Display RGB primaries and white point
2. Plot on CIE 1931 chromaticity diagram
3. Calculate triangle area
4. Compare to standard triangle area
5. Express as percentage coverage

**Requirements:**

| Grade | sRGB | DCI-P3 | Rec.2020 |
|-------|------|---------|----------|
| Standard | 95-100% | N/A | N/A |
| Wide Gamut | 100%+ | 90-100% | 70-85% |
| Professional | 100%+ | 95-100% | 80-90% |

---

## 3. Response Time Testing

### 3.1 GTG (Gray-to-Gray) Testing

**Equipment:**
- High-speed photodiode (rise time < 1µs)
- Oscilloscope or dedicated response time meter

**Measurement:**
1. Display 10% gray
2. Switch to 90% gray
3. Measure time from 10% to 90% of transition
4. Repeat for 90% to 10%
5. Average across multiple gray-to-gray pairs

**Transitions to Test:**
- 0% → 100% (black to white)
- 10% → 90%, 20% → 80%, etc.
- Worst-case transitions

**Standards:**

| Performance | GTG Time | Application |
|------------|---------|-------------|
| Slow | >10ms | General office |
| Standard | 5-10ms | Consumer |
| Fast | 3-5ms | Gaming (entry) |
| Very Fast | 1-3ms | Gaming (performance) |
| Extreme | <1ms | OLED, high-end gaming |

### 3.2 MPRT (Moving Picture Response Time)

**Equipment:**
- High-speed camera (120+ fps)
- Moving test pattern

**Procedure:**
1. Display moving pattern (scrolling bars, UFO test)
2. Capture with high-speed camera
3. Analyze motion blur in images
4. Calculate perceived response time

**Advantages:**
- Better correlation with user perception
- Accounts for sample-and-hold blur

**Reduction Techniques:**
- Black Frame Insertion (BFI)
- Backlight strobing
- Higher refresh rates (120Hz+)

### 3.3 Black-to-Black Testing

**Procedure:**
- 0% → 100% → 0% transition
- Measure total time

**Less Common:**
- Legacy metric
- GTG and MPRT more relevant

---

## 4. Contrast Ratio Testing

### 4.1 Static Contrast Ratio

**Measurement:**
1. Full white screen: Measure peak luminance
2. Full black screen: Measure minimum luminance
3. Calculate ratio: White / Black

**Formula:**
```
Contrast Ratio = Peak White Luminance / Black Luminance
```

**Technology Standards:**

| Technology | Typical Range | Notes |
|-----------|--------------|-------|
| TN LCD | 600:1 - 1,000:1 | Standard |
| IPS LCD | 1,000:1 - 1,500:1 | Good |
| VA LCD | 3,000:1 - 6,000:1 | Excellent |
| OLED | Infinite | 0 nits black |
| Mini-LED LCD | 10,000:1 - 100,000:1 | With local dimming |

**OLED Black Level:**
- Practical measurement: 0.0005 nits
- Limited by sensor noise floor
- Effectively infinite contrast

### 4.2 ANSI Contrast Ratio

**Pattern:**
- Checkerboard (16×16 black/white squares)
- More representative of real-world content

**Measurement:**
1. Measure white squares luminance
2. Measure black squares luminance
3. Average each set
4. Calculate ratio

**ANSI vs Static:**
- ANSI lower than static (realistic)
- Accounts for local light bleed, halation

---

## 5. Dead Pixel Testing

### 5.1 ISO 9241-3 Pixel Defect Classes

**Defect Types:**
- Type 1: Always on (stuck white/colored)
- Type 2: Always off (stuck black)
- Type 3: Subpixel defect (one subpixel stuck)

**Defect Classes:**

| Class | Type 1 | Type 2 | Type 3 | Cluster | Application |
|-------|--------|--------|--------|---------|-------------|
| I | 0 | 0 | 0 | 0 | Premium, medical |
| II | 2 | 2 | 5 | 0 | Standard consumer |
| III | 5 | 15 | 50 | 0 | Budget |
| IV | Not specified | Not specified | Not specified | Not specified | Industrial |

**Cluster Definition:**
- 5×5 pixel area
- Contains 2+ defects
- Unacceptable in Classes I-III

### 5.2 Testing Procedure

**Test Patterns:**
- Solid white (255,255,255)
- Solid black (0,0,0)
- Solid red (255,0,0)
- Solid green (0,255,0)
- Solid blue (0,0,255)

**Inspection:**
- Visual inspection (manual or automated)
- Dark room environment
- Multiple viewing angles
- Document defect locations

**Automated Detection:**
- Camera-based inspection systems
- Computer vision algorithms
- Factory production line integration
- Sub-second inspection times

---

## 6. HDR Testing and Certification

### 6.1 VESA DisplayHDR Standards

**DisplayHDR 400:**
- Peak Brightness: 400 nits (10% window)
- Black Level: 0.40 nits max (1,000:1 contrast)
- Color Gamut: 95% sRGB (CIE 1976)
- Bit Depth: 8-bit (with dithering) or 10-bit

**DisplayHDR 500:**
- Peak Brightness: 500 nits
- Black Level: 0.10 nits max (5,000:1 contrast)
- Color Gamut: 90% DCI-P3

**DisplayHDR 600:**
- Peak Brightness: 600 nits
- Black Level: 0.10 nits max
- Color Gamut: 99% DCI-P3

**DisplayHDR 1000:**
- Peak Brightness: 1,000 nits
- Black Level: 0.05 nits max (20,000:1 contrast)
- Color Gamut: 90% Rec.2020
- Local Dimming: Required

**DisplayHDR 1400:**
- Peak Brightness: 1,400 nits
- Highest tier certification

**DisplayHDR True Black (OLED-specific):**
- DisplayHDR 400 True Black: 400 nits peak, 0.0005 nits black
- DisplayHDR 500 True Black: 500 nits peak, 0.0005 nits black

### 6.2 HDR Testing Procedure

**Equipment:**
- Spectroradiometer (required for certification)
- HDR test patterns (VESA provided)

**Tests:**
1. Peak brightness (10% window, sustained)
2. Black level measurement
3. Contrast ratio calculation
4. Color gamut coverage
5. Color volume (3D gamut × brightness)
6. Tone mapping quality (visual assessment)

**Tone Mapping:**
- Display must properly handle HDR metadata
- Gradual highlights roll-off (no clipping)
- Shadow detail preservation

---

## 7. Refresh Rate and VRR Testing

### 7.1 Fixed Refresh Rate Validation

**Equipment:**
- High-speed camera (240+ fps) or oscilloscope
- Test pattern with frame counter

**Procedure:**
1. Display incrementing frame counter
2. Capture with high-speed camera
3. Count frames over known time period
4. Validate against specification (60Hz, 120Hz, etc.)

**Tolerance:**
- ±0.5% of nominal rate
- No dropped or duplicated frames

### 7.2 Variable Refresh Rate (VRR) Testing

**Adaptive Sync / FreeSync / G-Sync:**
- Test across advertised range (e.g., 48-144Hz)
- Verify smooth transitions between rates
- Check for artifacts during rate changes

**Test Procedure:**
1. Drive display at varying frame rates
2. Monitor actual refresh rate (oscilloscope or software)
3. Verify synchronization with source
4. Assess visual artifacts (tearing, stuttering)

**LFC (Low Framerate Compensation):**
- Test below minimum VRR range
- Verify frame doubling/tripling implementation

### 7.3 Flicker Testing

**PWM Backlight Flicker (LCD):**
- High-speed photodiode or camera
- Measure modulation depth and frequency

**Standards (IEEE P1789):**
- Frequency < 90Hz: Modulation depth < 10%
- Frequency > 1kHz: Generally acceptable
- Flicker-free (DC dimming): Preferred

**OLED Flicker:**
- Generally flicker-free (DC driven)
- Some low-brightness pulsing (< 60Hz) on some panels

---

## 8. Environmental Testing

### 8.1 Temperature Testing

**Operating Range:**
- Consumer: 0°C to 40°C
- Industrial: -20°C to 60°C
- Automotive: -30°C to 85°C (ambient), junction to 125°C

**Tests:**
1. Cold start: Power on at minimum temperature
2. Hot operation: Continuous operation at maximum temperature
3. Temperature cycling: -20°C to +60°C, 10 cycles minimum

**Acceptance:**
- Full functionality across range
- No permanent damage or degradation

### 8.2 Humidity Testing

**Accelerated Aging:**
- 85°C / 85% RH (relative humidity)
- 1,000 hours minimum (automotive: 2,000+ hours)

**Procedure:**
1. Place display in environmental chamber
2. Apply power cycling (on 30min, off 30min)
3. Inspect periodically for defects

**Failure Modes:**
- Moisture ingress (OLED degradation)
- Adhesive delamination
- Electrical shorts

### 8.3 Mechanical Testing

**Drop Testing (Mobile):**
- Multiple orientations (face, back, edge, corner)
- Drop height: 1-1.5 meters
- Hard surface (concrete, tile)
- 10-20 drops minimum

**Acceptance:**
- Display functional after drops
- No cracks, dead pixels, or delamination
- Cosmetic damage acceptable (bezel, frame)

**Vibration (Automotive):**
- Sinusoidal vibration: 10-200Hz sweep
- Random vibration: Power spectral density spec
- Duration: Hours to days

**Foldable Durability:**
- Fold cycles: 200,000-500,000 minimum
- Environmental conditions during folding
- Inspect for crease degradation, pixel defects

---

## 9. Quality Assurance and Acceptance

### 9.1 Incoming Inspection

**Sample Size:**
- AQL (Acceptable Quality Level) sampling per MIL-STD-105E
- Critical defects: 0% tolerance
- Major defects: 1.5% AQL
- Minor defects: 4.0% AQL

**Inspection Points:**
1. Visual inspection (dead pixels, uniformity)
2. Dimensional check
3. Electrical test (power-on, basic functionality)
4. Documentation verification

### 9.2 Final QA Before Shipment

**100% Testing (High-Value):**
- Every unit tested (premium/professional displays)
- Automated test stands
- Data logging for traceability

**Sampling (Consumer):**
- Statistical sampling per production batch
- Critical parameters: 100% check
- Non-critical: Sample-based

**Test Report:**
- Brightness, contrast, color accuracy measurements
- Defect inspection results
- Serial number, date, inspector ID
- Pass/fail status with criteria

---

## 10. Calibration and Certification

### 10.1 Factory Calibration

**Professional Displays:**
- Individual unit calibration
- Color accuracy tuning (ΔE < 2)
- Uniformity compensation (Mura correction)
- Calibration certificate included

**Consumer Displays:**
- Batch calibration (representative sample)
- Standard settings applied to all units
- Some premium models: Individual calibration

### 10.2 Certification Programs

**VESA DisplayHDR:**
- Third-party testing required
- Annual recertification
- Logo licensing

**TCO Certified:**
- Sustainability and ergonomics
- Emissions, materials, recycling

**TÜV Rheinland:**
- Flicker-free
- Low blue light
- Eye comfort

**CalMAN / SpectraCal:**
- Professional calibration certification
- Color accuracy verification

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025 | Initial release |

---

**Copyright**: © 2025 SmileStory Inc. / WIA
**License**: 弘益人間 (Benefit All Humanity)
