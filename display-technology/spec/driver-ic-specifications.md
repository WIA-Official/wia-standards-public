# Display Driver IC Specifications Standard

**WIA-SEMI-008 Display Technology Standard**
**Version**: 1.0
**Last Updated**: 2025

---

## Overview

This specification defines requirements for Display Driver ICs (DDIs), Timing Controllers (TCONs), gate drivers, source drivers, and power management ICs for LCD and OLED display panels.

## Scope

Covers:
- Source driver ICs (column drivers)
- Gate driver ICs (row drivers)
- Timing controllers (TCONs)
- Power management ICs (PMICs)
- Integrated touch and display controllers

---

## 1. Source Driver IC Specifications

### 1.1 General Requirements

**Channel Count:**
- Mobile: 384-1536 channels per IC
- TV/Monitor: 384-768 channels per IC
- Automotive: 192-384 channels per IC

**Resolution Support:**
- 6-bit: 64 levels per channel (262K colors)
- 8-bit: 256 levels per channel (16.7M colors)
- 10-bit: 1024 levels per channel (1.07B colors)

### 1.2 Electrical Characteristics

**Output Voltage Range:**
- LCD: 0V to 10V typical
- OLED: Current mode (programmable range)

**DAC Accuracy:**
- ±0.5% or better across full range
- Integral Non-Linearity (INL): ±2 LSB max
- Differential Non-Linearity (DNL): ±1 LSB max

**Settling Time:**
- <2µs typical for full-scale transition
- Critical for high refresh rates

**Output Drive Capability:**
- Load capacitance: 30-100pF typical
- Slew rate: >5 V/µs

### 1.3 Timing Specifications

**Clock Frequency:**
- Mobile: Up to 100 MHz
- Desktop: Up to 200 MHz
- High refresh: Up to 500 MHz

**Sampling Rate:**
- Matches pixel clock
- Double/quad sampling for advanced modes

**Latency:**
- Data to output: <2 clock cycles
- Minimal for gaming/VRR applications

### 1.4 Interface Standards

**Digital Input:**
- MIPI DSI (mobile): 2/4 lane, up to 2.5 Gbps/lane
- LVDS: Single/Dual/Quad configurations
- eDP: 1/2/4 lane, HBR/HBR2/HBR3
- Parallel RGB: Legacy, simple panels

**Control Signals:**
- HSYNC, VSYNC, Data Enable
- Clock input (DCLK)
- Reset, power-down controls

---

## 2. Gate Driver IC Specifications

### 2.1 General Requirements

**Output Channels:**
- 1-8 outputs per IC (cascadable)
- Typically hundreds of ICs per large panel

**Output Voltage:**
- On-state (VGH): +20V to +40V typical
- Off-state (VGL): -5V to 0V typical
- Programmable via external references

### 2.2 Electrical Characteristics

**Output Current:**
- Peak: 50-200mA per channel
- Drives long gate lines (high capacitance)

**Rise/Fall Times:**
- <1µs typical
- Critical for minimizing ghosting

**Output Impedance:**
- Low impedance for fast transitions
- Uniform across all channels (±5%)

### 2.3 Timing Specifications

**Shift Register:**
- Clock frequency: 10-100 kHz (line rate)
- Cascade-able for large panels

**Gate Pulse Width:**
- Programmable: 1-50µs typical
- Matches panel TFT charging requirements

### 2.4 Gate Driver on Array (GOA)

**Integration:**
- Fabricated using panel TFT process
- Located along panel edge
- Eliminates external gate driver ICs

**Advantages:**
- Narrower bezels
- Lower cost (no separate ICs)
- Fewer connections

**Challenges:**
- Yield sensitivity
- Limited current capability vs discrete ICs

---

## 3. Timing Controller (TCON) Specifications

### 3.1 General Requirements

**Input Interfaces:**
- HDMI 1.4/2.0/2.1 receiver
- DisplayPort 1.2/1.4/2.0 receiver
- MIPI DSI receiver
- LVDS receiver
- Analog VGA (legacy)

**Output Interfaces:**
- Parallel to source drivers
- Point-to-point high-speed lanes
- Control signals to gate drivers

### 3.2 Image Processing Capabilities

**Scaler:**
- Resolution conversion (upscaling/downscaling)
- Multiple input resolutions support
- Bilinear/bicubic interpolation

**Frame Rate Conversion:**
- 24fps→60fps
- 60fps→120fps
- 3:2 pulldown for film content

**Color Processing:**
- Color space conversion (YUV↔RGB)
- Color gamut mapping
- 3D LUT (Look-Up Table) support
- Gamma correction (adjustable curves)

**Overdrive:**
- Response time acceleration (LCD)
- Lookup tables for optimal driving
- Reduces motion blur

**HDR Processing:**
- HDR10, HDR10+, Dolby Vision decoding
- Tone mapping to panel capabilities
- Metadata parsing and application

### 3.3 Advanced Features

**Local Dimming Control:**
- Zone management: 8-5000 zones
- Real-time content analysis
- Minimize halo artifacts

**MEMC (Motion Estimation/Motion Compensation):**
- Intermediate frame generation
- Smoother motion rendering
- 24fps→60fps, 60fps→120fps

**Mura Compensation:**
- Per-pixel or per-region correction
- Factory calibration data
- Real-time uniformity enhancement

**VRR (Variable Refresh Rate):**
- FreeSync, G-Sync compatible
- HDMI VRR support
- Seamless rate transitions

### 3.4 Performance Specifications

**Processing Latency:**
- <1 frame typical for basic processing
- <2 frames with MEMC enabled
- Gaming mode: Minimize latency

**Memory Requirements:**
- Line buffer: 1-4 lines
- Frame buffer: 1-2 frames (for MEMC, scaling)
- DRAM interface for large buffers

**Power Consumption:**
- 1-5W typical (varies by resolution, features)
- Power-saving modes for static content

---

## 4. Power Management IC Specifications

### 4.1 General Requirements

**Voltage Rails:**

**LCD PMICs:**
- VDDIO: 1.8V or 3.3V (±3%)
- VDD: 3.3V (±5%)
- AVDD: 5-12V (±2%)
- AVEE: -5 to -12V (±2%)
- VGH/VGL: Gate voltages (±2%)
- VCOM: Adjustable common voltage

**OLED PMICs:**
- VDDIO: 1.8V (±3%)
- VDD: 3.3V (±5%)
- ELVDD: 4-7V (±2%)
- ELVSS: Negative supply (±2%)
- VGH/VGL: Gate voltages (±2%)

### 4.2 Electrical Characteristics

**Efficiency:**
- Buck converters: >85% efficiency
- Boost converters: >85% efficiency
- LDOs: Application-dependent

**Output Current:**
- VDDIO/VDD: 100-500mA
- AVDD/AVEE: 50-200mA
- Backlight: 200mA-2A (LED strings)

**Ripple and Noise:**
- <50mV peak-to-peak (critical rails)
- <100mV (less critical rails)
- Low noise for analog circuits

### 4.3 Sequencing and Protection

**Power-Up Sequence:**
1. VDD, VDDIO (logic supplies)
2. AVDD (positive supply)
3. AVEE (negative supply - MUST come after AVDD)
4. VGH/VGL (gate voltages)
5. VCOM
6. Backlight (LCD) or enable (OLED)

**Timing:**
- Inter-rail delays: 1-10ms typical
- Total power-up: <500ms
- Violation protection (prevents panel damage)

**Protection Features:**
- Overvoltage protection (OVP)
- Overcurrent protection (OCP)
- Overtemperature protection (OTP)
- Undervoltage lockout (UVLO)
- Short-circuit protection

### 4.4 Backlight Driver (LCD)

**Topology:**
- Boost converter + current regulator
- String voltage: 12-40V (depends on LED count)

**Dimming Methods:**
- PWM dimming: Adjustable frequency (200Hz-20kHz)
- Analog dimming: Direct current control
- Hybrid: Combined for optimal performance

**Local Dimming:**
- Multiple independent channels (8-5000 zones)
- Per-zone current control
- Synchronization with image content

**Efficiency:**
- >90% at nominal brightness
- Adaptive voltage (minimize headroom)

---

## 5. Integrated Touch and Display Controllers

### 5.1 General Requirements

**Integration:**
- Touch sensing circuits integrated with DDI
- Saves PCB space
- Thinner module assembly

**Touch Technologies:**
- Capacitive touch (mutual or self-capacitance)
- In-cell (within display layers)
- On-cell (on top of display)

### 5.2 Touch Performance

**Sampling Rate:**
- 60-240 Hz (matches or exceeds display refresh)
- Higher for stylus support (up to 480 Hz)

**Touch Points:**
- Multi-touch: 5-10 simultaneous touches
- Hover detection: Optional

**Latency:**
- <10ms touch-to-display response
- Critical for gaming, drawing

**Accuracy:**
- ±0.5mm positional accuracy
- ±1° angle accuracy (stylus)

### 5.3 Power Management

**Active Touch:**
- 10-50mW typical

**Idle/AOD Touch:**
- <5mW
- Reduced sampling rate

**Noise Immunity:**
- Display noise rejection
- Charger noise filtering
- Water rejection algorithms

---

## 6. Automotive-Specific Requirements

### 6.1 Qualification Standards

**AEC-Q100:**
- Grade 2: -40°C to +105°C
- Grade 3: -40°C to +85°C
- Stress tests: HTOL, TC, HTS, etc.

**Functional Safety:**
- ISO 26262 ASIL-B or ASIL-D (depends on application)
- Built-in self-test (BIST)
- Redundancy for critical functions

### 6.2 Extended Requirements

**Operating Temperature:**
- -40°C to +105°C (ambient)
- Junction temperature: Up to 150°C

**Lifespan:**
- 15+ years design life
- Extended reliability testing

**EMI/EMC:**
- Stringent automotive standards
- Immunity to RF interference

**Supply Voltage:**
- Wide input range (6-16V typical)
- Load dump tolerance
- Reverse polarity protection

---

## 7. Testing and Validation

### 7.1 Electrical Tests

**DC Parameters:**
- Output voltage accuracy
- Output current capability
- Quiescent current
- Efficiency measurements

**AC Parameters:**
- Settling time
- Slew rate
- Clock jitter
- Signal integrity

**Sequencing:**
- Power-up/power-down timing
- Violation detection

### 7.2 Functional Tests

**Image Quality:**
- Uniformity (DAC accuracy validation)
- Linearity (grayscale steps)
- Color accuracy
- No crosstalk between channels

**Timing:**
- Clock frequency range
- Data setup/hold times
- Synchronization accuracy

**Interface Compliance:**
- MIPI DSI/LVDS/eDP protocol compliance
- Electrical specifications (voltage, timing)

### 7.3 Environmental Tests

**Temperature:**
- Operating range: -40°C to +85°C (automotive)
- Storage range: -55°C to +125°C

**Humidity:**
- 85°C / 85% RH accelerated aging
- 1000 hours minimum

**Mechanical:**
- Vibration (automotive)
- Shock tolerance

---

## 8. Compliance and Certification

### 8.1 Standards Compliance

**Industry Standards:**
- MIPI Alliance (DSI, DCS, DPI)
- VESA (DisplayPort, eDP)
- HDMI Licensing
- USB-IF (USB-C Alt Mode)

**Regional Certifications:**
- CE (Europe)
- FCC (USA)
- CCC (China)
- KC (Korea)

### 8.2 Documentation Requirements

**Datasheets:**
- Electrical characteristics
- Timing diagrams
- Application schematics
- Layout guidelines

**Application Notes:**
- Power sequencing
- Interface design
- Calibration procedures
- Troubleshooting guides

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025 | Initial release |

---

**Copyright**: © 2025 SmileStory Inc. / WIA
**License**: 弘益人間 (Benefit All Humanity)
