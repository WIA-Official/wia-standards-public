# WIA-PET-007 PHASE 2: DEVICE HARDWARE SPECIFICATIONS

**Version:** 1.0.0  
**Date:** 2025-12-25  
**Status:** Active Standard

---

## 1. Hardware Requirements Overview

PHASE 2 defines minimum hardware specifications for WIA-PET-007 compliant pet wearable devices, covering processors, sensors, connectivity, power systems, and physical design.

---

## 2. Core Processing Unit

### 2.1 Minimum MCU Specifications

| Specification | Minimum Requirement |
|---------------|---------------------|
| Architecture | 32-bit ARM Cortex-M4 or equivalent |
| Clock Speed | 80 MHz |
| RAM | 256 KB |
| Flash Memory | 4 MB (for firmware + data logging) |
| Power Modes | Sleep, Deep Sleep, Ultra-Low-Power |

### 2.2 Co-Processor (Optional)

For GPS/cellular devices:
- Dedicated GPS/GNSS processor
- Cellular modem with integrated baseband
- Low-power always-on processor for wake events

---

## 3. Sensor Requirements

### 3.1 Accelerometer (Required)

+----------------------------+----------------------------------+
| Resolution                 | 16-bit minimum                   |
| Range                      | ±16g                             |
| Sampling Rate              | 25-100 Hz                        |
| Noise Level                | < 150 µg/√Hz                     |
| Power Consumption          | < 100 µA                         |
| Accuracy                   | ±5% for step counting            |
+----------------------------+----------------------------------+

### 3.2 Heart Rate Sensor (Health Monitoring Devices)

**Technology:** Photoplethysmography (PPG)

| Parameter | Specification |
|-----------|--------------|
| LED Wavelengths | Green (525nm), Red (660nm), IR (940nm) |
| Sampling Rate | 25 Hz continuous or periodic |
| Accuracy (Resting) | ±2 bpm or ±3%, whichever greater |
| Accuracy (Active) | ±5 bpm or ±5%, whichever greater |
| Power Consumption | 8-15 mA during active measurement |
| Placement | Direct skin contact required |

### 3.3 Temperature Sensor (Required)

| Type | Specification |
|------|--------------|
| Technology | Digital thermistor or infrared |
| Range | 35°C to 42°C (95°F to 107.6°F) |
| Accuracy | ±0.2°C (±0.36°F) |
| Sampling Rate | Every 1-5 minutes |
| Calibration | Factory calibrated, periodic self-check |

### 3.4 GPS Module (Location Devices)

| Feature | Requirement |
|---------|-------------|
| Systems Supported | GPS + at least one of GLONASS/Galileo/BeiDou |
| Accuracy | 2.5m CEP in open sky |
| Time to First Fix | < 30s hot start, < 60s cold start |
| Update Rate | 1-10 Hz configurable |
| Assisted GPS | A-GPS support required |
| Sensitivity | -165 dBm tracking |

---

## 4. Connectivity Modules

### 4.1 Bluetooth Low Energy (Required)

+--------------------------------+------------------------------+
| Bluetooth Version              | BLE 5.0 or higher            |
| Range                          | 10m indoor, 50m outdoor LOS  |
| Data Rate                      | 1 Mbps minimum               |
| Power Consumption              | < 5 mA during connection     |
| Connection Interval            | 15-30 ms                     |
| MTU                            | 247 bytes (negotiated)       |
+--------------------------------+------------------------------+

### 4.2 Wi-Fi (Optional)

- IEEE 802.11 b/g/n (2.4 GHz required, 5 GHz optional)
- Used for bulk data sync when at home
- Background sync during charging

### 4.3 Cellular (GPS Trackers)

- 4G LTE or 5G support
- Global roaming capability
- Low-power mode for battery conservation
- Fallback to 2G/3G if available

---

## 5. Power System

### 5.1 Battery Specifications by Size Category

+------------+------------------+------------------+--------------------+
| Device     | Battery Type     | Capacity Range   | Min Battery Life   |
| Size       |                  |                  |                    |
+------------+------------------+------------------+--------------------+
| XS         | LiPo single cell | 100-200 mAh      | 5-7 days           |
| Small      | LiPo single cell | 200-400 mAh      | 7-10 days          |
| Medium     | LiPo single cell | 400-800 mAh      | 10-14 days         |
| Large      | LiPo multi-cell  | 800-1500 mAh     | 12-21 days         |
| XL         | LiPo multi-cell  | 1500-2500 mAh    | 14-28 days         |
+------------+------------------+------------------+--------------------+

### 5.2 Battery Protection (Mandatory)

- Overcharge protection: Hardware cutoff at 4.2V/cell
- Over-discharge protection: Cutoff at 2.8V/cell
- Temperature monitoring: Shutdown if > 60°C
- Short circuit protection: < 100ms disconnect
- Current limiting: Fuse or PTC resettable fuse

### 5.3 Charging System

**Wired Charging:**
- USB Type-C connector (USB 2.0 data + power)
- 5V DC input, 0.5-1A charging current
- Charge time: 0-80% in < 2 hours, 0-100% in < 3 hours

**Wireless Charging (Optional):**
- Qi standard 1.2.4 or higher
- 5W minimum power
- Foreign object detection required

---

## 6. Environmental Protection

### 6.1 Waterproof Rating

+------------+------------------------------------------+
| IP65       | Minimum (dust-tight, water jets)         |
| IP67       | Recommended (immersion to 1m / 30 min)   |
| IP68       | Premium (immersion beyond 1m)            |
+------------+------------------------------------------+

### 6.2 Operating Conditions

| Parameter | Range |
|-----------|-------|
| Operating Temperature | -10°C to 50°C |
| Storage Temperature | -20°C to 60°C |
| Humidity | 5% to 95% non-condensing |
| Drop Resistance | 1.5m onto concrete (IEC 60068-2-32) |

---

## 7. Physical Design

### 7.1 Size and Weight Constraints

+------------------+-----------------+------------------+
| Pet Weight Range | Max Device Wt.  | Max Dimensions   |
+------------------+-----------------+------------------+
| 0-2 kg           | 10g             | 30x25x12mm       |
| 2-5 kg           | 15g             | 40x30x15mm       |
| 5-10 kg          | 25g             | 50x35x18mm       |
| 10-25 kg         | 40g             | 60x40x20mm       |
| 25-45 kg         | 60g             | 70x45x22mm       |
| 45+ kg           | 80g             | 80x50x25mm       |
+------------------+-----------------+------------------+

### 7.2 Ergonomic Requirements

- Rounded edges: Minimum 2mm radius on all corners
- No sharp protrusions or edges
- Balanced weight distribution
- Low profile to prevent snagging

---

## 8. Material Safety

### 8.1 Approved Materials

**Enclosure:**
- Medical-grade silicone (USP Class VI)
- Food-grade TPU, PC, ABS, or PETG
- Must be BPA-free, phthalate-free

**Strap/Band:**
- Medical-grade silicone
- Nylon webbing (non-toxic dyes)
- Polyester (food-safe)

**Metal Components:**
- Stainless steel 316L (nickel-free)
- Titanium

### 8.2 Biocompatibility Testing

All pet-contact materials must pass:
- ISO 10993-5: Cytotoxicity
- ISO 10993-10: Sensitization and Irritation
- ISO 10993-11: Systemic Toxicity

---

## 9. User Interface

### 9.1 LED Indicators

+-------------------+-------------------------------+
| Status            | LED Pattern                   |
+-------------------+-------------------------------+
| Battery > 50%     | Green solid                   |
| Battery 20-50%    | Yellow solid                  |
| Battery < 20%     | Red solid                     |
| Battery < 10%     | Red blinking                  |
| BLE Connected     | Blue solid                    |
| BLE Searching     | Blue blinking                 |
| Charging          | Green pulsing                 |
| Error             | Red rapid blink               |
+-------------------+-------------------------------+

### 9.2 Buttons

- Primary button: Power on/off, mode selection
- Waterproof, tactile feedback
- 5+ year lifecycle rating
- Optional: Emergency SOS button

---

## 10. Electromagnetic Compatibility

### 10.1 Emissions

- FCC Part 15 (USA)
- CE EMC Directive EN 55032 (Europe)
- Harmonic distortion limits

### 10.2 Immunity

- ESD: ±8kV contact, ±15kV air
- RF immunity: 10 V/m (80 MHz - 6 GHz)
- Surge protection: ±1kV line-to-line

---

## 11. RF Radiation Safety

### 11.1 SAR Limits

- Maximum SAR: 1.6 W/kg (1g tissue average)
- Minimize transmission duty cycle
- Antenna placement away from body when possible

---

## 12. Manufacturing Quality

### 12.1 Production Testing

Every device must undergo:
- Electrical parameter testing (±5% tolerance)
- Sensor calibration verification
- RF performance testing
- Waterproof pressure testing
- Firmware functional testing
- Battery capacity verification (>95% rated)

### 12.2 Quality Standards

- ISO 9001: Quality management certification
- Component traceability via serial numbers
- Failure analysis for all RMA units

---

## 13. Modular Design and Repairability

### 13.1 Replaceable Components

- User-replaceable battery (tools allowed)
- Replaceable straps/bands (standard attachment)
- Modular sensor pods (manufacturer service)

### 13.2 Spare Parts

- Availability: Minimum 5 years post-production
- Repair manuals for authorized service centers

---

## Appendix A: Reference Bill of Materials (BOM)

+----------------------------+-------------------------+------------------+
| Component                  | Example Part            | Quantity         |
+----------------------------+-------------------------+------------------+
| MCU                        | Nordic nRF52840         | 1                |
| Accelerometer              | Bosch BMA400            | 1                |
| PPG Sensor                 | Maxim MAX30102          | 1                |
| Temperature Sensor         | TI TMP117               | 1                |
| GPS Module                 | u-blox NEO-M9N          | 1                |
| LiPo Battery               | 3.7V 500mAh             | 1                |
| Charging IC                | TI BQ25895              | 1                |
| Flash Memory               | Winbond W25Q32          | 1                |
| LED                        | Multi-color SMD         | 2                |
| PCB                        | 4-layer FR-4            | 1                |
| Enclosure                  | Silicone overmold       | 1                |
+----------------------------+-------------------------+------------------+

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA  
WIA-PET-007 PHASE 2 Specification
