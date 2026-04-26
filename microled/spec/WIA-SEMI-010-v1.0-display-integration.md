# WIA-SEMI-010 v1.0: Display Integration Standards

## Document Information

- **Standard**: WIA-SEMI-010
- **Version**: 1.0
- **Date**: 2025-01-15
- **Category**: System Integration

## 1. Scope

This specification defines requirements for integrating MicroLED chips into complete display systems, including TFT backplanes, drivers, and optical components.

## 2. TFT Backplane Specifications

### 2.1 TFT Technologies

Supported technologies:
- LTPS (Low-Temperature Polysilicon)
- Oxide TFT (IGZO)
- LTPO (Hybrid LTPS+Oxide)
- CMOS Silicon

### 2.2 Electrical Requirements

**TFT Performance**:
- Electron mobility: >10 cm²/V·s (Oxide), >100 cm²/V·s (LTPS)
- On/off ratio: >10⁶
- Threshold voltage variation: <±0.3V across panel
- Leakage current: <1pA at off-state

**Current Drive Capability**:
- Minimum drive current per pixel: 1μA to 10mA depending on application
- Current uniformity: <±5% across panel

## 3. Pixel Circuit Designs

### 3.1 Minimum Circuit Configurations

**2T1C** (Simple):
- Select TFT
- Drive TFT
- Storage capacitor

**4T2C** (Compensated):
- Select, Drive, Compensation, Emission control TFTs
- Storage and Compensation capacitors

### 3.2 Pixel Pitch

| Application | Pixel Pitch |
|-------------|-------------|
| Smartphones | 20-80μm |
| TVs | 0.4-2.0mm |
| AR Microdisplays | 3-10μm |
| Large Format | 0.6-3.0mm |

## 4. Driver IC Integration

### 4.1 Source Driver Requirements

- Resolution: 8-12 bits per color
- Output channels: 1,000-3,000 per IC
- Output voltage range: 0-10V
- Update rate: >120 Hz

### 4.2 Gate Driver Requirements

- Output voltage: -7V to +15V typical
- Channels: 1,000-4,000 per IC
- Rise/fall time: <1μs

### 4.3 Timing Controller (TCON)

- Interface: LVDS, eDP, MIPI, HDMI
- Image processing: Scaling, color correction, uniformity compensation
- Memory: Sufficient for calibration data storage

## 5. Optical Specifications

### 5.1 Display Performance

**Brightness**:
- Smartphones: 800-1,200 nits typical, 1,500+ nits peak
- TVs: 600-2,000 nits
- Automotive: 1,000-10,000 nits (HUD)
- AR: 5,000-10,000 nits

**Contrast Ratio**: >1,000,000:1 (pixel-level)

**Color Gamut**: >90% DCI-P3, >85% Rec. 2020 target

**Viewing Angle**: >170° at 50% brightness

### 5.2 Uniformity

- Brightness uniformity: <5% variation (9-point measurement)
- Color uniformity: ΔE <2 (CIEDE2000)
- Mura: Not visible at normal viewing distance

## 6. Thermal Management

### 6.1 Temperature Limits

- Operating: 0°C to +60°C
- Storage: -20°C to +80°C
- Junction temperature: <85°C recommended, <100°C maximum

### 6.2 Thermal Design

- Thermal resistance: Junction to ambient <10 K/W per watt
- Heat spreaders: Graphite, copper, or aluminum
- Thermal interface materials: Thermal conductivity >1 W/m·K

## 7. Encapsulation and Protection

### 7.1 Moisture Barrier

- WVTR (Water Vapor Transmission Rate): <1×10⁻⁶ g/m²/day
- OTR (Oxygen Transmission Rate): <1×10⁻⁵ cm³/m²/day

### 7.2 Mechanical Protection

- Cover glass thickness: 0.3-1.1mm
- Pencil hardness: >6H
- Impact resistance: Per IEC 60068-2-75

## 8. Calibration

### 8.1 Factory Calibration

- Brightness calibration: All pixels to <±5% variation
- Color calibration: White point to D65 ±0.003 u'v'
- Gamma calibration: Gamma 2.2 or 2.4 <±5% error

### 8.2 Calibration Data Storage

- Non-volatile memory in TCON or system
- Data retention: >10 years
- Update capability for field recalibration

## 9. Quality and Reliability

### 9.1 Pixel Defect Standards (ISO 9241-305)

**Class I** (Premium):
- Dead pixels: 0
- Bright pixels: 0

**Class II** (Standard):
- Dead pixels: <2 per million
- Bright pixels: <2 per million

### 9.2 Lifetime

- Display lifetime: >50,000 hours to 80% brightness
- TFT lifetime: >100,000 hours operation
- Driver IC lifetime: >50,000 hours

## 10. Environmental

- RoHS compliant
- REACH compliant
- Energy Star qualified (where applicable)
- Recyclable materials preferred

## 11. Testing

### 11.1 Electrical Testing

- Open/short circuit test: 100% coverage
- Pixel functionality: 100% coverage
- Power consumption verification

### 11.2 Optical Testing

- Brightness uniformity measurement
- Color uniformity measurement
- Viewing angle characterization

### 11.3 Environmental Testing

- Temperature cycling: -20°C to +60°C
- Humidity: 85°C/85% RH, 500 hours
- Vibration: Per IEC 60068-2-6

## 12. Documentation

Required documentation:
- Schematic diagrams
- Calibration procedures
- Test reports
- Material declarations

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
