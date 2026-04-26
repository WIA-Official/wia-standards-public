# WIA-SEMI-009: OLED/LED Measurement Protocols
## Detailed Measurement Specification v1.0

**Document Type:** Technical Measurement Standard  
**Related Standard:** WIA-SEMI-009 Core v1.0  
**Release Date:** 2025-01-15  

---

## 1. General Measurement Principles

### 1.1 Measurement Environment

**Ambient Conditions:**
- Temperature: 25°C ± 2°C
- Humidity: 50% ± 10% RH
- Atmospheric pressure: 86-106 kPa

**Lighting:**
- Dark room: <1 lux ambient light (for contrast, black level measurements)
- Controlled lighting: 64 lux (for color accuracy measurements per ISO 3664)
- Light source: D65 or D50 (specify which)

**Display Warm-up:**
- Minimum 30 minutes operation before measurement
- Display test pattern during warm-up
- Thermal equilibrium required

### 1.2 Equipment Calibration

**Colorimeters and Spectroradiometers:**
- Calibration: Annual minimum, NIST-traceable
- Verification: Weekly against reference standard
- Measurement geometry: 2° or 10° (CIE standard observer)

**Photometers:**
- Luminance range: 0.01 - 10,000 Cd/m²
- Accuracy: ±2% or ±0.01 Cd/m², whichever greater
- Linearity: <1% deviation

---

## 2. Luminance and Brightness Measurement

### 2.1 Full-Screen White Luminance

**Test Pattern:**
- RGB values: 255, 255, 255 (8-bit) or maximum for bit depth
- Pattern: Full screen uniform white
- Duration: 5 minutes to reach thermal steady-state

**Measurement Points:**
- Center point (mandatory)
- 9-point grid (recommended): Center + midpoints of 4 edges + 4 corners
- 25-point grid (professional/certification): 5×5 uniform grid

**Calculation:**
- Average luminance = Sum of measurements / Number of points
- Uniformity = (Max - Min) / Average × 100%

**Acceptance Criteria:**
- Consumer: Uniformity <15%
- Professional: Uniformity <10%
- Reference: Uniformity <5%

### 2.2 Peak Brightness Measurement

**Test Patterns:**

**3% Window:**
- White rectangle: 10% width × 30% height (= 3% area)
- Positioned at screen center
- Background: Black (RGB 0, 0, 0)

**10% Window:**
- White rectangle: 31.6% width × 31.6% height (= 10% area)
- Positioned at screen center
- Background: Black

**Measurement:**
- Wait 30 seconds for ABL stabilization
- Measure center of white window
- Record as "Peak Brightness (3%)" or "Peak Brightness (10%)"

### 2.3 Black Level Measurement

**Test Pattern:**
- Full screen black (RGB 0, 0, 0)
- Display for 5 minutes before measurement

**Measurement Requirements:**
- Dark room: <0.1 lux ambient
- Photometer sensitivity: capable of measuring <0.01 Cd/m²
- For OLED: verify display is truly off (no backlight leakage)

**Procedure:**
1. Display full black pattern
2. Measure ambient light contribution with display off
3. Turn display on (black pattern)
4. Measure luminance
5. Black level = On measurement - Ambient contribution

---

## 3. Contrast Ratio Measurement

### 3.1 ANSI Contrast (Recommended)

**Test Pattern:**
- 16×16 checkerboard
- Even squares: White (RGB 255, 255, 255)
- Odd squares: Black (RGB 0, 0, 0)
- Square size: 1/16 of screen width/height

**Measurement:**
1. Display checkerboard pattern
2. Measure luminance at center of 8 white squares
3. Measure luminance at center of 8 black squares
4. Calculate Average(White) and Average(Black)
5. ANSI Contrast = Average(White) / Average(Black)

**Recommended White Square Positions:**
- (3,3), (5,5), (7,7), (9,9), (11,11), (13,13), (5,11), (11,5)
  where (x,y) are square coordinates (1-16)

**Recommended Black Square Positions:**
- (4,4), (6,6), (8,8), (10,10), (12,12), (14,14), (6,12), (12,6)

### 3.2 Sequential Contrast

**Procedure:**
1. Display full white, measure luminance (Lwhite)
2. Display full black, measure luminance (Lblack)
3. Sequential Contrast = Lwhite / Lblack

**Note:** Sequential contrast typically higher than ANSI contrast due to local dimming optimization, but less representative of real-world performance.

### 3.3 Native Contrast (LCD)

**For LCD with Local Dimming:**
1. Disable local dimming feature
2. Measure full white luminance
3. Measure full black luminance
4. Native Contrast = White / Black

**Purpose:** Characterizes LCD panel capability independent of backlight control.

---

## 4. Color Measurement

### 4.1 Color Gamut Measurement

**Primary Colors Test:**

**Pattern:**
- Display full-screen red (RGB 255, 0, 0)
- Display full-screen green (RGB 0, 255, 0)
- Display full-screen blue (RGB 0, 0, 255)
- Display white (RGB 255, 255, 255)

**Measurement:**
- Use spectroradiometer (preferred) or colorimeter
- Measure CIE 1931 xy chromaticity coordinates
- Or CIE 1976 u'v' coordinates (for perceptual uniformity)

**Gamut Calculation:**
1. Plot measured primaries on CIE diagram
2. Calculate triangle area
3. Compare to target color space (sRGB, DCI-P3, Rec.2020)
4. Coverage % = (Display Gamut Area) / (Target Gamut Area) × 100%

**Standard Color Spaces:**

| Color Space | Red (x,y) | Green (x,y) | Blue (x,y) | White (x,y) |
|-------------|-----------|-------------|------------|-------------|
| sRGB | 0.640, 0.330 | 0.300, 0.600 | 0.150, 0.060 | 0.3127, 0.3290 (D65) |
| DCI-P3 | 0.680, 0.320 | 0.265, 0.690 | 0.150, 0.060 | 0.314, 0.351 (D65) |
| Rec.2020 | 0.708, 0.292 | 0.170, 0.797 | 0.131, 0.046 | 0.3127, 0.3290 (D65) |

### 4.2 Color Accuracy Measurement

**Test Patterns:**
- ColorChecker 24-patch chart (standard)
- Or CalMAN, Displaycal patterns
- Minimum 20 color patches covering gamut

**Procedure:**
1. Display each color patch at specified luminance (typically 100-120 Cd/m²)
2. Measure with spectroradiometer
3. Calculate ΔE (Delta E) for each patch
4. Use ΔE2000 formula (recommended) or ΔE76 (legacy)

**ΔE2000 Calculation:**
Complex formula accounting for perceptual non-uniformity of color space.
Refer to CIE publication or use calibrated software.

**Reporting:**
- Average ΔE2000 across all patches
- Maximum ΔE2000
- Number of patches with ΔE > 3.0
- Color error histogram

**Acceptance Criteria:**
- Professional: Average ΔE < 2.0, Max ΔE < 3.0
- Premium Consumer: Average ΔE < 3.0, Max ΔE < 5.0
- Standard Consumer: Average ΔE < 5.0, Max ΔE < 8.0

### 4.3 White Point Measurement

**Test:**
- Display full white at multiple luminance levels (e.g., 50, 100, 200, 400 Cd/m²)
- Measure CCT (Correlated Color Temperature) and Δuv (deviation from Planckian locus)

**Target:**
- CCT: 6500K ± 300K (D65 standard)
- Δuv: ±0.002 (professional), ±0.005 (consumer)

---

## 5. Temporal Measurement

### 5.1 Response Time (Gray-to-Gray)

**Equipment:**
- High-speed photometer: 1kHz+ sample rate
- Positioned to measure single pixel area if possible
- Stable mount to avoid vibration

**Test Procedure:**
1. Display initial gray level (e.g., 0, 25, 50, 75, 100%)
2. Switch to target gray level
3. Record luminance vs. time
4. Determine transition time (10%-90% of change)

**Recommended Transitions:**
- 0% → 100% (Black to White)
- 100% → 0% (White to Black)
- 25% → 75% (Gray to Gray)
- 50% → 80% (Gray to Gray)
- 80% → 50% (Gray to Gray)

**Reporting:**
- Average GtG response time
- Slowest GtG response time
- Rise time vs. fall time (if different)

### 5.2 Input Lag Measurement

**Equipment:**
- High-speed camera (240fps minimum)
- Reference display with known low lag
- Synchronized input signal

**Procedure:**
1. Connect test display and reference display to same signal source
2. Trigger simultaneous image change
3. Record both displays with high-speed camera
4. Count frame difference
5. Input lag = Frame difference / Camera frame rate × 1000 ms

**Acceptance:**
- Gaming: <10ms
- Professional: <15ms
- Consumer: <30ms

### 5.3 Motion Clarity (MPRT)

**MPRT (Moving Picture Response Time):**

**Equipment:**
- Pursuit camera or specialized motion analysis system

**Procedure:**
1. Display moving pattern (e.g., scrolling white bar on black background)
2. Capture with pursuit camera (tracking motion)
3. Analyze blur length
4. MPRT = blur length / pattern speed × 1000

**Factors:**
- Pixel response time
- Refresh rate
- Backlight behavior (PWM, strobe)

---

## 6. OLED-Specific Measurements

### 6.1 Lifetime Testing Protocols

**Accelerated Aging Test:**

**Conditions:**
- Temperature: 60°C ambient (display reaches 65-70°C)
- Luminance: 500 Cd/m² (full white) or application-specific
- Pattern: Full white or application pattern
- Duration: 1,000-5,000 hours minimum

**Measurements:**
- Initial characterization (t=0): Luminance, color, efficiency
- Periodic measurements: Every 100-500 hours
- Parameters tracked: Luminance decay, color shift, efficiency change

**Lifetime Prediction:**
1. Fit luminance vs. time to exponential decay: L(t) = L₀ × exp(-t/τ)
2. Extrapolate to LT95 (95% of L₀)
3. Apply acceleration factor to predict normal-use lifetime

**Acceleration Factor:**
- Temperature: Use Arrhenius equation
  - AF_temp = exp((Ea/k) × (1/T_use - 1/T_test))
  - Ea: Activation energy (determined empirically, typically 0.3-1.0 eV)
- Luminance: Power law
  - AF_lum = (L_test / L_use)^n
  - n: Acceleration factor (typically 1.5-2.0)

### 6.2 Burn-in Assessment

**Static Image Test:**
1. Display static pattern (e.g., logo, UI) at center
2. Brightness: typical use level (200-400 Cd/m²)
3. Duration: 500-2,000 hours
4. Periodic uniformity check: every 100-250 hours

**Uniformity Measurement:**
1. Display uniform gray (50% white)
2. Measure 25-point grid luminance
3. Calculate uniformity deviation
4. Observe for ghost image of static pattern

**Acceptance:**
- After 500h: Uniformity deviation <5% (premium), <8% (standard)
- After 1,000h: Uniformity deviation <8% (premium), <12% (standard)
- No visible ghost image on uniform gray

---

## 7. LED Backlight Measurements

### 7.1 Backlight Uniformity

**Test:**
- Full white display (all pixels maximum transmission)
- Measure 25-point grid
- Calculate uniformity

**Uniformity Metric:**
- Uniformity % = (Lmax - Lmin) / Lavg × 100%

**Acceptance:**
- Edge-lit: <15%
- Direct-lit: <10%
- Mini-LED: <8% (500+ zones), <5% (2000+ zones)

### 7.2 Local Dimming Zone Characterization

**Zone Isolation Test:**
1. Turn on single zone to maximum
2. All other zones off
3. Measure luminance profile across display
4. Characterize zone size, blooming, crosstalk

**Blooming Quantification:**
- Measure halo extent (distance from zone center where luminance <10% of peak)
- Halo luminance (max luminance in halo region)

**Acceptance:**
- Halo extent <1.5× nominal zone size
- Halo luminance <10% of peak white

### 7.3 PWM Flicker

**Measurement:**
- High-speed photometer or oscilloscope with photodiode
- Sample rate >10× PWM frequency

**Analysis:**
- FFT to determine fundamental frequency
- Calculate flicker index: (Max - Min) / (Max + Min)

**Acceptance:**
- PWM frequency >240Hz (minimum), >480Hz (preferred)
- Flicker index <0.05 (for PWM >480Hz)

---

## 8. Quality Control Sampling

### 8.1 Sampling Plans

**Production Sampling:**
- Incoming materials: AQL 0.65 (critical defects)
- Process monitoring: 100% key parameters, sampled comprehensive
- Final inspection: 100% functional, sampled optical/lifetime

**Lot Size Based Sampling (per ANSI/ASQ Z1.4):**
| Lot Size | Sample Size (Normal Inspection, Level II, AQL 1.0) |
|----------|---------------------------------------------------|
| 2-8 | 2 |
| 9-15 | 3 |
| 16-25 | 5 |
| 26-50 | 8 |
| 51-90 | 13 |
| 91-150 | 20 |
| 151-280 | 32 |
| 281-500 | 50 |

### 8.2 Defect Classification

**Critical Defects:** Render display non-functional or unsafe
- Complete display failure
- Safety hazard (fire, shock, excessive light)
- No image output

**Major Defects:** Significantly impair performance
- Dead pixels (>5 per million pixels)
- Color uniformity >20% deviation
- Luminance <50% of specification

**Minor Defects:** Cosmetic or minor performance issues
- Single dead pixel
- Slight color tint (<ΔE 5)
- Minor uniformity issues (10-15%)

---

## 9. Uncertainty and Error Analysis

### 9.1 Measurement Uncertainty

**Sources:**
- Instrument calibration: ±2%
- Environmental variation: ±1%
- Display stability: ±2%
- Spatial variation: ±3%

**Combined Uncertainty:**
- Root Sum Square (RSS): √(2² + 1² + 2² + 3²) ≈ ±4.2%

**Reporting:**
- Measurement results should include uncertainty
- Example: "Luminance: 500 ± 21 Cd/m² (k=1)"

### 9.2 Repeatability and Reproducibility

**Repeatability (same operator, same equipment):**
- Coefficient of variation <3%

**Reproducibility (different operators, different equipment):**
- Coefficient of variation <5%

**Verification:**
- Regular inter-laboratory comparisons
- Round-robin testing with reference samples

---

## 10. Reporting Requirements

### 10.1 Test Report Contents

**Minimum Required Information:**
1. Display identification (model, serial number, manufacturer)
2. Test date and location
3. Operator and equipment used
4. Environmental conditions
5. Test patterns and procedures
6. Measurement results with uncertainty
7. Pass/fail determination with acceptance criteria
8. Deviations from standard (if any)

### 10.2 Data Format

**Numerical Precision:**
- Luminance: 1 Cd/m² or 0.1% (whichever larger)
- Color coordinates: 0.001 (xy or u'v')
- ΔE: 0.1
- Response time: 0.1 ms

**Units:**
- Luminance: Cd/m² or nit
- Color: CIE 1931 xy or CIE 1976 u'v'
- Time: ms
- Temperature: °C

---

**Published by:**  
World Certification Industry Association (WIA)  
弘益人間 · Benefit All Humanity

© 2025 SmileStory Inc. / WIA
