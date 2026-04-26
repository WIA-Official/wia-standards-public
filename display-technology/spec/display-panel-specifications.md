# Display Panel Specifications Standard

**WIA-SEMI-008 Display Technology Standard**
**Version**: 1.0
**Last Updated**: 2025

---

## Overview

This specification defines standardized parameters, measurement methods, and requirements for LCD, OLED, AMOLED, and MicroLED display panels across mobile, desktop, TV, and automotive applications.

## Scope

This standard applies to all flat-panel displays used in:
- Mobile devices (smartphones, tablets)
- Desktop monitors
- Television displays
- Automotive displays
- Wearables and AR/VR devices
- Signage and industrial displays

---

## 1. Resolution and Pixel Density Specifications

### 1.1 Resolution Standards

**Common Resolutions:**

| Name | Resolution | Aspect Ratio | Total Pixels | Applications |
|------|-----------|--------------|--------------|--------------|
| HD | 1280×720 | 16:9 | 921,600 | Budget devices |
| FHD (1080p) | 1920×1080 | 16:9 | 2,073,600 | Standard |
| QHD | 2560×1440 | 16:9 | 3,686,400 | Premium mobile |
| 4K UHD | 3840×2160 | 16:9 | 8,294,400 | TV, monitors |
| 5K | 5120×2880 | 16:9 | 14,745,600 | Professional |
| 8K UHD | 7680×4320 | 16:9 | 33,177,600 | Future TV |

**Mobile Specific:**
- FHD+: 2340×1080, 2400×1080 (19.5:9, 20:9)
- QHD+: 3200×1440, 3120×1440 (20:9, 19.5:9)

### 1.2 PPI (Pixels Per Inch) Standards

**Formula:**
```
PPI = √(width² + height²) / diagonal_inches
```

**Standards by Application:**
- **Budget Mobile**: 200-300 PPI
- **Standard Mobile**: 300-400 PPI (Retina threshold)
- **Premium Mobile**: 400-600+ PPI
- **Desktop Monitors**: 90-150 PPI
- **4K TV (55")**: ~80 PPI
- **Professional Monitors**: 200-300 PPI

### 1.3 Subpixel Arrangements

**RGB Stripe:**
- Standard layout: R-G-B-R-G-B
- Best for text rendering
- 100% effective resolution
- Used in: LCD, high-end OLED

**PenTile (RGBG):**
- Samsung AMOLED layout
- Shared subpixels (2 green per R/B)
- ~70% effective horizontal resolution
- Power efficiency advantages

**WRGB (LG WOLED):**
- White, Red, Green, Blue subpixels
- White for brightness boost
- Used in LG OLED TVs

---

## 2. Brightness and Contrast Specifications

### 2.1 Brightness (Luminance)

**Measurement:**
- Unit: cd/m² (nits)
- Condition: Full white screen, center measurement
- Warm-up: 30 minutes minimum

**Standard Brightness Levels:**

| Application | SDR (nits) | HDR Peak (nits) |
|------------|-----------|----------------|
| Budget Mobile | 300-400 | N/A |
| Premium Mobile | 500-800 | 1,000-1,800 |
| Desktop Monitor | 250-350 | N/A |
| HDR Monitor | 400+ | 400-1,400 |
| TV (SDR) | 300-500 | N/A |
| TV (HDR) | 400-700 | 600-2,000+ |

**Sustained vs Peak:**
- Peak: Short-duration maximum (10% window, HDR highlights)
- Sustained: Full-screen continuous brightness
- OLED ABL (Automatic Brightness Limiting) reduces peak for large areas

### 2.2 Contrast Ratio

**Definition:**
```
Contrast Ratio = Peak White Luminance / Black Luminance
```

**Standards by Technology:**

**LCD:**
- TN Panel: 600:1 - 1,000:1
- IPS Panel: 1,000:1 - 1,500:1
- VA Panel: 3,000:1 - 6,000:1
- Mini-LED LCD: 10,000:1 - 100,000:1 (effective with local dimming)

**OLED:**
- Static: Infinite (0.0005 nits black level)
- Practical: 1,000,000:1 (limited by measurement equipment)

### 2.3 Uniformity

**Brightness Uniformity:**
- Measurement: 9-point or 13-point grid
- Calculation: (1 - ((Max - Min) / Max)) × 100%
- Target: > 90% for professional, > 85% for consumer

**Color Uniformity:**
- Delta E < 3 across panel (professional: ΔE < 2)
- Measured at multiple points across screen

---

## 3. Color Specifications

### 3.1 Color Gamut Coverage

**Standard Requirements:**

| Grade | sRGB | DCI-P3 | Rec.2020 | Applications |
|-------|------|---------|----------|--------------|
| Basic | 60-80% | N/A | N/A | Budget |
| Standard | 95-100% | 70-85% | 50-60% | General |
| Wide Gamut | 100%+ | 90-100% | 75-85% | Professional |
| Reference | 100%+ | 100%+ | 85-95% | Color-critical |

### 3.2 Color Accuracy

**Delta E (ΔE) Standards:**
- ΔE < 3: Acceptable for consumer use
- ΔE < 2: Professional standard
- ΔE < 1: Reference/medical imaging
- ΔE < 0.5: High-end professional

**Measurement:**
- Colorimeter or spectrophotometer
- 24-patch ColorChecker or equivalent
- Calibrated state after 30-min warm-up

### 3.3 White Point and Gamma

**White Point:**
- D65 (6500K): Standard for video/computing
- D50 (5000K): Print production
- DCI (6300K): Cinema projection

**Gamma:**
- 2.2: Computer displays standard
- 2.4: Video in dark environments
- 2.6: Cinema projection
- ST.2084 (PQ): HDR content

**Tolerance:**
- Professional: ±200K white point, ±0.05 gamma
- Consumer: ±500K white point, ±0.1 gamma

---

## 4. Response Time and Refresh Rate

### 4.1 Response Time Standards

**GTG (Gray-to-Gray):**
- Measurement: Time for 10%→90% and 90%→10% transitions
- Fast: < 5ms
- Very Fast: < 3ms
- Extreme: < 1ms (TN, Fast IPS, OLED)

**MPRT (Moving Picture Response Time):**
- Perceptual motion clarity metric
- Better correlation with user experience
- Reduction via BFI, backlight strobing, high refresh rates

**Black-to-Black:**
- Legacy metric
- Less relevant for modern displays
- OLED: <0.1ms typical

### 4.2 Refresh Rate

**Standard Rates:**
- 60 Hz: Standard displays
- 75 Hz: Entry gaming
- 90 Hz: Mobile premium
- 120 Hz: Gaming/premium mobile
- 144 Hz: Gaming monitors
- 165/180/240/360 Hz: High-end gaming

**Variable Refresh Rate (VRR):**
- Adaptive Sync / FreeSync: VESA standard
- G-Sync: NVIDIA proprietary (compatible mode available)
- HDMI VRR: HDMI 2.1 specification
- Range: Typically 48-144Hz or wider

---

## 5. Power Consumption Specifications

### 5.1 Power Metrics

**Measurement Conditions:**
- Standard content (typical use case)
- Maximum brightness
- Minimum brightness
- Power-saving modes

**Typical Power Consumption:**

| Type | Size | Power (W) | Notes |
|------|------|-----------|-------|
| Smartphone LCD | 6" | 2-4W | Active use |
| Smartphone OLED | 6" | 1-5W | Content dependent |
| Laptop LCD | 15" | 5-15W | Variable brightness |
| Desktop Monitor | 27" | 30-60W | Standard use |
| 4K TV LCD | 55" | 100-200W | SDR content |
| OLED TV | 55" | 80-300W | Content dependent |

### 5.2 Power Efficiency

**Efficacy (Luminance per Watt):**
- cd/m² per Watt
- Higher is more efficient
- Varies by technology, brightness level

**LTPO Benefits:**
- 15-20% power savings vs standard LTPS
- Enabled by 1-120Hz variable refresh
- Critical for mobile battery life

---

## 6. Viewing Angle Specifications

### 6.1 Measurement Standard

**Definition:**
Angle from normal where contrast ratio drops to 10:1 or luminance drops by 50%.

**Technology Performance:**

| Technology | Horizontal | Vertical | Notes |
|-----------|-----------|----------|-------|
| TN LCD | 90° | 65° | Significant color shift |
| IPS LCD | 178° | 178° | Minimal color shift |
| VA LCD | 178° | 178° | Gamma shift at angles |
| OLED | 180° | 180° | Minimal shift |

### 6.2 Color Shift Tolerance

**Professional Standard:**
- ΔE < 3 at ±45° horizontal/vertical
- Minimal brightness variation

**Consumer Standard:**
- Usable image at ±60° horizontal
- Some color/brightness shift acceptable

---

## 7. Durability and Reliability

### 7.1 Lifespan

**Measurement:**
Time to 50% original brightness (Half-life)

**Standards:**

| Technology | Half-Life (hours) | Equivalent Years (8h/day) |
|-----------|-------------------|--------------------------|
| LCD (backlight) | 50,000-100,000 | 17-34 years |
| OLED (blue) | 30,000-100,000 | 10-34 years |
| MicroLED | 100,000+ | 34+ years |

### 7.2 Environmental Operating Conditions

**Temperature:**
- Operating: 0°C to 40°C (consumer)
- Automotive: -30°C to 85°C
- Industrial: -20°C to 60°C

**Humidity:**
- Operating: 10% to 90% RH (non-condensing)
- Storage: 5% to 95% RH

### 7.3 Mechanical Durability

**Foldable Displays:**
- Fold cycles: 200,000-500,000 minimum
- Fold radius: 1.4-5mm (current)
- Crease visibility: Minimized but present

**Scratch Resistance:**
- Mohs hardness: 6-7 (glass)
- Coating hardness: 3H-9H (pencil hardness scale)

---

## 8. Testing and Certification

### 8.1 Required Tests

**Visual Quality:**
1. Brightness uniformity (9-point measurement)
2. Color uniformity (ΔE across panel)
3. Dead pixel inspection (ISO 9241-3)
4. Viewing angle performance
5. Response time (GTG, MPRT)

**Environmental:**
1. Temperature cycling
2. Humidity/moisture resistance
3. Drop testing (mobile devices)
4. Vibration testing (automotive)

**Electrical:**
1. Power consumption
2. EMI/EMC compliance
3. ESD tolerance

### 8.2 Certifications

**HDR:**
- VESA DisplayHDR 400/500/600/1000/1400
- VESA DisplayHDR True Black (OLED)

**Environmental:**
- TCO Certified (sustainability)
- EPEAT (environmental rating)
- Energy Star

**Safety:**
- TÜV Rheinland (flicker-free, eye comfort)
- Blue light filtering certifications

---

## 9. Compliance and Traceability

### 9.1 Documentation Requirements

All display panels must include:
- Detailed specification sheet
- Test reports (brightness, color, response time)
- Calibration data (if applicable)
- Manufacturing date and lot number
- Compliance certifications

### 9.2 Quality Control

**Acceptance Criteria:**
- Brightness: ±10% of specification
- Color accuracy: Within specified ΔE tolerance
- Dead pixels: Per ISO 9241-3 class
- Uniformity: Within specification range

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025 | Initial release |

---

**Copyright**: © 2025 SmileStory Inc. / WIA
**License**: 弘益人間 (Benefit All Humanity)
