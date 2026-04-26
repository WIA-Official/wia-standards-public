# WIA-SEMI-009: Mini-LED Backlight Specification
## Mini-LED and Local Dimming Standard v1.0

**Document Type:** Technology-Specific Standard  
**Related Standard:** WIA-SEMI-009 Core v1.0  
**Release Date:** 2025-01-15  

---

## 1. Mini-LED Definition and Classification

### 1.1 LED Size Classification

**Mini-LED:** LED chip size 100-500μm
**Micro-LED:** LED chip size <100μm (typically for self-emissive displays)
**Traditional LED:** LED chip size >500μm (typically 1-3mm)

### 1.2 Mini-LED Backlight Classification

**Class 1 - Entry Mini-LED:**
- LED count: 500-2,000
- Dimming zones: 100-500
- LED chip size: 300-500μm

**Class 2 - Standard Mini-LED:**
- LED count: 2,000-8,000
- Dimming zones: 500-2,000
- LED chip size: 200-400μm

**Class 3 - Premium Mini-LED:**
- LED count: 8,000-15,000
- Dimming zones: 2,000-5,000
- LED chip size: 100-300μm

**Class 4 - Ultra Mini-LED:**
- LED count: >15,000
- Dimming zones: >5,000
- LED chip size: 100-200μm

---

## 2. LED Component Specifications

### 2.1 LED Chip Requirements

**Luminous Efficacy:**
- Minimum: 130 lm/W at 350mA
- Target: 150-200 lm/W

**Forward Voltage:**
- Typical: 2.8-3.2V at 350mA
- Binning tolerance: ±0.1V within batch

**Color Temperature (For White LEDs):**
- Target: 6500K ± 300K (D65)
- Binning: 3-step MacAdam ellipse or better

**Viewing Angle:**
- >120° at 50% luminous intensity

### 2.2 LED Binning and Matching

**Luminous Flux:**
- Bin width: ±10% of nominal value
- All LEDs in single display from same bin

**Color:**
- Chromaticity bins: 3-step MacAdam ellipse maximum
- Preferably 2-step for premium products

**Forward Voltage:**
- Maximum spread within display: 0.2V

---

## 3. Backlight Architecture

### 3.1 LED Arrangement

**Grid Pattern (Most Common):**
- Uniform rectangular grid
- Spacing: typically 5-15mm (varies by LED count and panel size)

**Hexagonal Pattern:**
- Close-packed hexagonal array
- More efficient area coverage
- Complex driver routing

**Adaptive Pattern:**
- Higher density in center
- Lower density at edges
- Optimized for typical viewing content

### 3.2 Optical Distance (OD)

**Definition:** Distance from LED surface to diffuser sheet

**Traditional Direct-Lit:** 15-30mm
**Mini-LED:** 8-20mm
**OD Zero Mini-LED:** 3-10mm

**Trade-offs:**
- Smaller OD → Thinner backlight, potential hotspots
- Larger OD → Better uniformity, thicker backlight

### 3.3 Optical Films Stack

**Minimum Configuration:**
1. Reflector plate (behind LEDs)
2. Primary diffuser (closest to LEDs)
3. Secondary diffuser
4. Brightness enhancement film (BEF) or equivalent
5. Reflective polarizer (optional but recommended)

**Premium Configuration:**
1. Reflector with micro-structures
2. Primary diffuser with gradient scattering
3. Micro-lens array (for OD zero)
4. Secondary diffuser
5. Dual BEF (DBEF)
6. Reflective polarizer
7. Quantum dot film (if used)

---

## 4. Local Dimming Specifications

### 4.1 Zone Definition

**Zone:** Group of LEDs controlled together, or single LED if individually controlled

**Zone Shape:**
- Rectangular: Most common, simple algorithm
- Hexagonal: Better natural coverage
- Arbitrary/Adaptive: Complex but optimal

**Zone Count Minimum:**
| Display Size | Minimum Zones (Entry) | Recommended (Premium) |
|--------------|----------------------|----------------------|
| <32" | 64 | 500+ |
| 32-43" | 100 | 1,000+ |
| 43-55" | 200 | 2,000+ |
| 55-65" | 300 | 3,000+ |
| 65-75" | 400 | 4,000+ |
| >75" | 500 | 5,000+ |

### 4.2 Dimming Algorithm Requirements

**Minimum Features:**
- Per-zone brightness control based on image content
- Temporal filtering to prevent flicker
- Spatial filtering to reduce zone boundaries

**Recommended Features:**
- Blooming prediction and mitigation
- Content-aware optimization
- HDR metadata utilization (if HDR supported)
- Predictive algorithms (next-frame anticipation)

**Performance:**
- Zone update rate: Match or exceed panel refresh rate
- Latency: <1 frame
- Flicker: Not perceptible (<0.05 flicker index)

### 4.3 Blooming Performance

**Blooming Test:**
- Display single white pixel or 1% white window on black background
- Measure halo size and luminance

**Acceptance Criteria:**

| Mini-LED Class | Max Halo Extent | Max Halo Luminance |
|----------------|----------------|-------------------|
| Class 1 (Entry) | 2× zone size | 15% of peak white |
| Class 2 (Standard) | 1.5× zone size | 10% of peak white |
| Class 3 (Premium) | 1.2× zone size | 5% of peak white |
| Class 4 (Ultra) | 1.1× zone size | 3% of peak white |

---

## 5. Driver and Control Electronics

### 5.1 LED Driver IC Specifications

**Current Control:**
- Accuracy: ±1% channel-to-channel
- Resolution: 12-bit minimum (4096 levels), 14-bit preferred
- Temperature compensation required

**Channels per IC:**
- Typical: 16-48 channels
- Higher channel count reduces IC count but increases routing complexity

**PWM Specifications:**
- Frequency: >480Hz minimum, >960Hz preferred
- Frequency stability: ±1%
- Phase alignment: synchronize all ICs to prevent beat frequency

### 5.2 Communication Interface

**From Main Processor to Driver ICs:**
- SPI, I2C, or proprietary high-speed serial
- Data rate: sufficient for zone count × refresh rate × bit depth
- Example: 5,000 zones × 120 Hz × 12 bits = 7.2 Mbps minimum

**Latency:**
- Communication + processing: <8.3ms (1 frame at 120Hz)

### 5.3 Power Supply

**Voltage:**
- Typical LED forward voltage: 2.8-3.2V
- Driver supply: 24-48V DC typical

**Current:**
- Total current capability: (LED count) × (max current per LED) + 20% margin
- Example: 10,000 LEDs × 60mA × 1.2 = 720A at LED voltage

**Efficiency:**
- Power supply efficiency: >90%
- Driver IC efficiency: >85%

**Power Management:**
- Dynamic power allocation (ABL for Mini-LED)
- Thermal monitoring and throttling
- Soft-start capability

---

## 6. Uniformity and Performance

### 6.1 Luminance Uniformity

**Full White Uniformity:**
- All zones at 100%
- Measure 25-point grid

**Acceptance:**
- Class 1: <10% deviation
- Class 2: <8% deviation
- Class 3: <6% deviation
- Class 4: <5% deviation

### 6.2 Color Uniformity

**Measurement:**
- Full white display
- Measure chromaticity (u', v') at 9 or 25 points

**Acceptance:**
- Δu'v' < 0.004 (consumer)
- Δu'v' < 0.002 (professional)

**Correction:**
- Per-zone color correction recommended for premium products
- Requires RGB LED or multi-channel driver

### 6.3 Contrast Performance

**Native LCD Contrast:**
- IPS: >800:1
- VA: >2,500:1

**With Local Dimming:**
- ANSI Contrast (16×16 checkerboard):
  - Class 1: >10,000:1
  - Class 2: >20,000:1
  - Class 3: >50,000:1
  - Class 4: >80,000:1

---

## 7. Quantum Dot Integration

### 7.1 QD Film Placement

**Between Backlight and LCD:**
- QD film converts blue LED to red + green + blue
- Barrier films required (protect QD from moisture/oxygen)
- Typical thickness: 50-200μm

**On-Glass (QDOG):**
- QD layer integrated into LCD panel
- Thinner overall
- Better optical coupling

### 7.2 QD Performance Requirements

**Conversion Efficiency:**
- Blue to red: >60% quantum efficiency
- Blue to green: >70% quantum efficiency

**Color Gamut with QD:**
- Minimum: 95% DCI-P3
- Target: 98-100% DCI-P3

**Lifetime:**
- LT95 >30,000 hours under operating conditions

---

## 8. Thermal Management

### 8.1 Thermal Requirements

**LED Junction Temperature:**
- Maximum: 85°C (continuous operation)
- Recommended: <75°C for extended lifetime

**Backlight Surface Temperature:**
- Maximum: 60°C (accessible surface)

### 8.2 Thermal Design

**Heat Sink:**
- Material: Aluminum (typical) or copper (premium)
- Thermal interface material (TIM) between LEDs and heat sink
- Thermal resistance: <5°C/W per watt of LED power

**Thermal Monitoring:**
- Temperature sensors in backlight (minimum 4, one per quadrant)
- Thermal throttling algorithm
- Over-temperature shutdown (>90°C)

---

## 9. Reliability and Lifetime

### 9.1 LED Lifetime

**LT70 Definition:** Time to 70% of initial luminance

**Requirements:**
- Minimum: 50,000 hours LT70 at rated conditions
- Target: 100,000 hours LT70

**Accelerated Testing:**
- High temperature operation (85°C junction)
- Elevated current (1.5× rated)
- Extrapolate using standard models

### 9.2 Driver Reliability

**MTBF (Mean Time Between Failures):**
- Driver ICs: >100,000 hours
- Power supply: >50,000 hours

**Failure Mode:**
- Graceful degradation preferred (single LED failure doesn't affect others)
- Redundancy in critical circuits

---

## 10. Testing and Quality Control

### 10.1 Incoming LED Testing

**Sampling:**
- 100% testing for critical parameters (Vf, flux) or statistical sampling with tight binning

**Parameters:**
- Forward voltage at rated current
- Luminous flux
- Color coordinates
- Visual inspection (no cracks, proper die attachment)

### 10.2 Backlight Assembly Testing

**100% Tests:**
- Visual inspection (no dead LEDs, uniformity screen)
- Electrical (no shorts, proper current draw)

**Sample Tests (per lot):**
- Full characterization (uniformity, color, brightness)
- Aging test (100-500 hours burn-in)
- Vibration and shock (for automotive)

### 10.3 Final Display Testing

**Optical Tests:**
- Luminance and uniformity
- Contrast ratio (ANSI)
- Color gamut and accuracy
- Blooming assessment

**Functional Tests:**
- Local dimming operation
- Power consumption
- Thermal performance

---

## 11. Environmental and Safety

### 11.1 Operating Conditions

**Temperature:**
- Operating: 0-40°C ambient (consumer), -20-60°C (automotive)
- Storage: -40-85°C

**Humidity:**
- Operating: 10-90% RH, non-condensing
- Storage: 5-95% RH, non-condensing

### 11.2 Blue Light Safety

**IEC 62471 Classification:**
- Target: Risk Group 0 (Exempt) or Risk Group 1 (Low Risk)
- Measurement: Weighted blue light hazard

**Mitigation:**
- Blue light filter mode (reduce blue LED output)
- Adaptive brightness (lower at night)

---

**Published by:**  
World Certification Industry Association (WIA)  
弘益人間 · Benefit All Humanity

© 2025 SmileStory Inc. / WIA
