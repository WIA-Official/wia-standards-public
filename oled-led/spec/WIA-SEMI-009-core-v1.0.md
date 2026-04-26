# WIA-SEMI-009: OLED/LED Display Technology Standard
## Core Specification v1.0

**Document Status:** Official Standard  
**Release Date:** 2025-01-15  
**Effective Date:** 2025-02-01  
**Supersedes:** None (Initial Release)  

---

## 1. Scope and Purpose

### 1.1 Scope

This standard defines comprehensive specifications, measurement methodologies, and quality requirements for OLED (Organic Light-Emitting Diode) and LED (Light-Emitting Diode) display illumination technologies, including:

- WOLED (White OLED with color filters)
- QD-OLED (Quantum Dot OLED)  
- Tandem OLED (Multi-stack OLED)
- Mini-LED backlit LCD
- Traditional LED backlit LCD
- Emerging technologies (Micro-LED, QD-EL)

### 1.2 Purpose

- Establish standardized measurement protocols for display performance
- Define minimum performance requirements for various applications
- Ensure reproducibility and comparability across manufacturers
- Provide guidance for testing, qualification, and certification
- Support global supply chain quality assurance

### 1.3 Applicable Products

- Consumer displays (TVs, monitors, smartphones, tablets)
- Professional displays (broadcast, medical, CAD/design)
- Automotive displays (instrument clusters, infotainment, HUDs)
- Commercial displays (signage, retail, public information)
- Specialty displays (AR/VR, wearables, industrial equipment)

### 1.4 Related Standards

- WIA-SEMI-001: Semiconductor Device Standards
- WIA-SEMI-010: Display Driver IC Standards
- VESA DisplayHDR Specifications
- IEC 61966-2-1: sRGB Color Space
- ITU-R BT.2020: UHDTV Color Gamut
- DCI P3: Digital Cinema Color Space

---

## 2. Terminology and Definitions

### 2.1 Luminance and Brightness

**Luminance (Cd/m² or nit):** Photometric measure of luminous intensity per unit area in a given direction. SI unit: candela per square meter (Cd/m²). Informally called "nit" (equivalent units).

**Peak Brightness:** Maximum luminance achievable by display under specified conditions, typically measured on small white window (3%, 10% of screen area).

**Full-Screen Brightness:** Luminance when entire display area shows white, typically lower than peak due to thermal and power limitations.

**ABL (Automatic Brightness Limiter):** Dynamic brightness reduction based on average picture level to manage power and heat.

### 2.2 Contrast Ratio

**Contrast Ratio:** Ratio of brightest white to darkest black luminance.
- Native Contrast: Panel capability without local dimming
- Dynamic Contrast: With local dimming or adaptive brightness
- ANSI Contrast: Measured using checkerboard pattern (more realistic)

**Infinite Contrast:** When black level is truly zero (OLED), mathematically infinite but reported as ">1,000,000:1" for practical purposes.

### 2.3 Color Metrics

**Color Gamut:** Range of colors display can reproduce, typically expressed as percentage of standard color space (sRGB, DCI-P3, Rec.2020).

**Color Accuracy (ΔE):** Difference between target and measured color.
- ΔE < 1: Imperceptible difference
- ΔE < 2: Excellent, professional-grade
- ΔE < 3: Good, acceptable for critical work
- ΔE > 5: Noticeable color inaccuracy

**Color Volume:** 3D representation of color gamut across luminance range.

### 2.4 Temporal Characteristics

**Response Time (ms):** Time for pixel to change from one gray level to another, typically measured gray-to-gray (GtG).

**Refresh Rate (Hz):** Number of times per second display updates image.

**Input Lag (ms):** Delay between input signal and display update.

### 2.5 OLED-Specific Terms

**LT95:** Lifetime until luminance degrades to 95% of initial value.

**Burn-in:** Permanent (or semi-permanent) image retention from uneven pixel aging.

**Tandem OLED:** Two or more emission units stacked vertically with charge generation layers between.

### 2.6 LED-Specific Terms

**Local Dimming:** Capability to independently control brightness of backlight zones.

**FALD (Full-Array Local Dimming):** Direct-lit LED backlight with many zones distributed across panel area.

**Blooming:** Halo effect around bright objects on dark background due to imperfect zone isolation.

**Mini-LED:** LED chips 100-500μm, enabling thousands of dimming zones.

**Micro-LED:** LED chips <100μm, typically self-emissive (not backlight).

---

## 3. Performance Specifications

### 3.1 Luminance Requirements

| Application | Min Full-Screen | Min Peak (10%) | Max ABL Reduction |
|-------------|-----------------|----------------|-------------------|
| Budget TV | 150 Cd/m² | 300 Cd/m² | 50% |
| Mid-Range TV | 200 Cd/m² | 500 Cd/m² | 40% |
| Premium TV | 250 Cd/m² | 1,000 Cd/m² | 30% |
| HDR TV | 300 Cd/m² | 1,500 Cd/m² | 25% |
| Smartphone | 400 Cd/m² | 800 Cd/m² | 30% |
| Monitor (Professional) | 250 Cd/m² | 400 Cd/m² | 10% |
| Automotive | 300 Cd/m² | 1,500 Cd/m² | 20% |

### 3.2 Contrast Specifications

| Display Type | Min Native Contrast | Min ANSI Contrast | Black Level |
|--------------|---------------------|-------------------|-------------|
| IPS LCD | 800:1 | 600:1 | <0.3 Cd/m² |
| VA LCD | 2,500:1 | 1,500:1 | <0.1 Cd/m² |
| Mini-LED (500+ zones) | 3,000:1 | 10,000:1 | <0.05 Cd/m² |
| Mini-LED (2000+ zones) | 5,000:1 | 50,000:1 | <0.02 Cd/m² |
| OLED | Infinite | >100,000:1 | <0.0005 Cd/m² |

### 3.3 Color Performance

**Minimum Color Gamut Coverage:**
| Application | sRGB | DCI-P3 | Rec.2020 |
|-------------|------|--------|----------|
| Budget | 95% | 70% | - |
| Mid-Range | 100% | 85% | 60% |
| Premium | 100% | 95% | 70% |
| Professional | 100% | 98% | 75% |

**Maximum Color Error (ΔE2000):**
- Professional: ΔE < 2.0 average, ΔE < 3.0 maximum
- Consumer Premium: ΔE < 3.0 average, ΔE < 5.0 maximum
- Consumer Standard: ΔE < 5.0 average, ΔE < 8.0 maximum

### 3.4 Temporal Performance

**Response Time Requirements:**
| Application | Max GtG Response | Max Input Lag |
|-------------|------------------|---------------|
| Gaming | 5ms | 10ms |
| Professional | 10ms | 15ms |
| General Consumer | 15ms | 30ms |

**Refresh Rate Capabilities:**
- Standard: 60Hz minimum
- Gaming: 120Hz minimum, 144-240Hz preferred
- Professional: 60Hz minimum, 10-bit color

### 3.5 Efficiency Specifications

**Luminous Efficacy (lm/W):**
- LED Backlit LCD: >100 lm/W system efficiency
- Mini-LED: >80 lm/W system efficiency
- OLED: >25 lm/W effective (>80 lm/W OLED before filters)

**Power Consumption (55" Display, Typical Content):**
- Budget LCD: <100W
- Premium LCD/Mini-LED: <150W
- OLED: <120W (content-dependent)

---

## 4. Measurement Methodologies

### 4.1 Luminance Measurement

**Equipment:**
- Colorimeter or Spectroradiometer
- Calibrated to CIE 1931 standard observer
- Measurement angle: 2° or 10° (specify which)

**Procedure:**
1. Warm up display for 30 minutes
2. Set ambient temperature: 25±2°C
3. Display pattern: Full white (RGB 255,255,255)
4. Measure center point luminance
5. For peak brightness: Use 3% or 10% white window
6. Report ambient light conditions

**9-Point Uniformity:**
- Measure at 9 points (center + 8 edge/corner)
- Calculate uniformity: (Max-Min)/Average × 100%
- Maximum uniformity deviation: <15% (consumer), <10% (professional)

### 4.2 Contrast Measurement

**ANSI Contrast (16×16 Checkerboard):**
1. Display 16×16 checkerboard (white/black squares)
2. Measure luminance of 8 white squares
3. Measure luminance of 8 black squares
4. Contrast = Average(White) / Average(Black)

**Sequential Contrast:**
1. Measure full-screen white luminance
2. Measure full-screen black luminance
3. Contrast = White / Black

Note: ANSI contrast more representative of real-world performance.

### 4.3 Color Measurement

**Gamut Measurement:**
1. Display primary colors at maximum saturation
2. Measure CIE 1931 xy or CIE 1976 u'v' coordinates
3. Plot gamut triangle
4. Calculate coverage of target color space

**Color Accuracy:**
1. Display color patches from test pattern (e.g., ColorChecker)
2. Measure each patch with spectroradiometer
3. Calculate ΔE2000 for each patch
4. Report average and maximum ΔE

### 4.4 Response Time

**Gray-to-Gray Method:**
1. Transition pixel from Gray(L1) to Gray(L2)
2. Measure luminance over time with high-speed photometer (1kHz+ sample rate)
3. Response time = time from 10% to 90% of final luminance
4. Test multiple gray levels (recommended: 0-100%, 25-75%, 50-80%)

---

## 5. OLED-Specific Requirements

### 5.1 Lifetime Testing

**Accelerated Aging:**
- Temperature: 60°C ± 5°C (accelerated) or 25°C (normal)
- Luminance: 200-1000 Cd/m² (specify)
- Pattern: Full white or specific application pattern
- Measurement frequency: Hourly or continuous

**Acceptance Criteria:**
- LT95 > 50,000 hours (consumer)
- LT95 > 100,000 hours (automotive, professional)

**Acceleration Factor:**
- Must validate acceleration model
- Arrhenius equation for temperature
- Power law for luminance/current

### 5.2 Burn-in Testing

**Static Pattern Test:**
1. Display static image (logo, UI elements) at typical brightness
2. Duration: 1,000-5,000 hours
3. Periodically display uniform gray to assess retention
4. Measure uniformity deviation

**Acceptance:**
- Uniformity deviation <5% after 1,000h (premium)
- Uniformity deviation <10% after 1,000h (standard)

**Mitigation Features (Recommended):**
- Pixel shift capability
- Logo detection and dimming
- Compensation algorithms
- Periodic refresh cycles

---

## 6. LED/Mini-LED Specific Requirements

### 6.1 Backlight Uniformity

**Measurement:**
- Display full white
- Measure 25-point grid across panel
- Calculate uniformity

**Requirements:**
| Backlight Type | Max Deviation |
|----------------|---------------|
| Edge-lit | ±15% |
| Direct-lit | ±10% |
| Mini-LED (500+ zones) | ±8% |
| Mini-LED (2000+ zones) | ±5% |

### 6.2 Local Dimming Performance

**Zone Count:**
- Specify number of independently controllable zones
- Minimum for "local dimming" classification: 16 zones
- Mini-LED classification: >200 zones

**Blooming Test:**
1. Display single white pixel or small object on black background
2. Measure halo size and luminance
3. Calculate blooming index

**Acceptable Blooming:**
- Halo should not extend >20% beyond nominal zone size
- Halo luminance <10% of peak white

### 6.3 Dimming Protocol

**PWM Specifications:**
- Frequency: >240Hz minimum (>480Hz preferred)
- Duty cycle range: 1-100%
- Flicker index: <0.05

**Communication:**
- Define interface between driver IC and backlight
- Update rate: Match or exceed panel refresh rate
- Latency: <1 frame

---

## 7. Quality Assurance and Testing

### 7.1 Incoming Material Testing

**For OLED Materials:**
- Organic material purity: >99.9%
- Shelf life validation
- Performance characterization (efficiency, color, lifetime)

**For LED Chips:**
- Forward voltage binning (±0.1V)
- Luminous flux binning (±10%)
- Color binning (3-step MacAdam ellipse or better)

### 7.2 Process Control

**OLED Deposition:**
- Vacuum level: <10^-6 torr
- Deposition rate: ±5% tolerance
- Thickness monitoring: ±2% accuracy

**LED Placement (Mini-LED):**
- Position accuracy: ±10μm
- Bond strength: >5N per LED
- Dead pixel rate: <0.001%

### 7.3 Final Panel Testing

**100% Testing:**
- Pixel functionality (all pixels tested)
- Luminance measurement (center point minimum)
- Color check
- Uniformity assessment

**Sampling Testing:**
- Full characterization on sample (e.g., 1 in 100)
- Lifetime prediction
- Environmental testing (temperature, humidity)

---

## 8. Certification and Compliance

### 8.1 WIA-SEMI-009 Certification Levels

**Level 1 - Basic Compliance:**
- Meets minimum luminance and contrast specs
- Color gamut >90% sRGB
- Basic testing performed

**Level 2 - Standard Compliance:**
- Meets standard specifications for application
- Color gamut >95% DCI-P3
- Comprehensive testing

**Level 3 - Premium Compliance:**
- Exceeds standard specs by >20%
- Professional color accuracy (ΔE<2)
- Extended lifetime validation

### 8.2 Certification Process

1. Submit specification sheet and test data
2. WIA review of documentation
3. On-site or third-party lab validation testing
4. Certificate issuance (valid 2 years)
5. Ongoing surveillance (annual)

### 8.3 Certification Marks

Certified products may display:
- "WIA-SEMI-009 Certified" mark
- Certification level (1, 2, or 3)
- Expiration date

---

## 9. Environmental and Safety

### 9.1 Materials Restrictions

**RoHS Compliance:**
- Lead <1000 ppm (except exemptions)
- Cadmium <100 ppm (including QDs)
- Mercury prohibited

**REACH Compliance:**
- Substances of Very High Concern (SVHC) <0.1% w/w

### 9.2 Energy Efficiency

**Energy Star Requirements:**
- On-mode power: Must meet Energy Star specifications for size class
- Standby power: <0.5W
- Power management: Auto-brightness, sleep mode required

### 9.3 Safety Standards

- IEC 62471: Photobiological safety of lamps and lamp systems
- IEC 60950-1 or IEC 62368-1: Safety of IT equipment
- Blue light exposure: Category 0 or 1 (exempt or low risk)

---

## 10. Revision History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | 2025-01-15 | Initial release | WIA Standards Committee |

---

**Published by:**  
World Certification Industry Association (WIA)  
弘益人間 · Benefit All Humanity

**Contact:**  
Website: https://wiabooks.store/tag/wia-oled-led/  
GitHub: https://github.com/WIA-Official/wia-standards  

© 2025 SmileStory Inc. / WIA. All rights reserved.
