# WIA-SEMI-016 Integration Guidelines

## Best Practices for Electronic Skin System Integration

**Version**: 1.0
**Date**: 2025-01-01

---

## 1. System Architecture

### 1.1 Typical E-Skin System Components

```
[E-Skin Sensor Array]
        ↓
[Signal Conditioning]
        ↓
[ADC / Signal Processing]
        ↓
[Microcontroller / Processor]
        ↓
[Wireless Communication] ←→ [External Device/Gateway]
        ↓                          ↓
[Power Management]          [Cloud/Application]
```

### 1.2 Design Considerations

**Modularity**: Design sensor layer, electronics, and power as separable modules
**Scalability**: Architecture should support 10-1000+ sensors
**Redundancy**: Critical applications should have backup sensors/circuits
**Serviceability**: Plan for calibration, battery replacement, firmware updates

---

## 2. Electrical Integration

### 2.1 Sensor Readout Circuits

**Resistive Sensors** (Piezoresistive, Strain Gauges):
- Voltage divider or Wheatstone bridge configuration
- Constant voltage or constant current excitation
- Differential amplification to reject common-mode noise
- Low-pass filtering (fc = 100-500 Hz typical)

**Capacitive Sensors**:
- AC excitation (1-100 kHz)
- Capacitance-to-voltage or capacitance-to-digital converter
- Shield electrodes to reduce parasitic capacitance
- Examples: AD7746, FDC2214 ICs

### 2.2 Multiplexing Strategies

**Passive Matrix**:
- N rows × M columns = N×M sensors with N+M connections
- Sequential scanning: Select row, read all columns
- Maximum scan rate limited by sensor response time
- Crosstalk mitigation: Use high-impedance switches

**Active Matrix**:
- Transistor at each pixel enables individual addressing
- Requires flexible TFT technology
- Higher cost but eliminates crosstalk
- Faster readout possible

**Frequency Multiplexing**:
- Different sensors modulated at different frequencies
- All signals combined, separated by filters in readout
- Reduces wiring but requires careful filter design

### 2.3 Analog-to-Digital Conversion

**ADC Selection Criteria**:
- Resolution: 12-16 bits typical (10 bits minimum)
- Sampling rate: ≥2× highest signal frequency (100-1000 Hz per channel)
- Channels: Multi-channel ADC or multiplexer + single ADC
- Interface: SPI or I2C for microcontroller connection

**Recommended ADCs**:
- ADS1015/1115 (I2C, 12/16-bit, 4 channel)
- MCP3008 (SPI, 10-bit, 8 channel)
- Integrated in MCU (STM32, nRF52 series)

---

## 3. Processing and Algorithms

### 3.1 Signal Processing Pipeline

1. **Baseline Correction**: Subtract zero-load baseline
2. **Filtering**: Digital low-pass (Butterworth, Chebyshev) or moving average
3. **Calibration**: Apply stored calibration curve (linear, polynomial, or lookup table)
4. **Feature Extraction**: Peak detection, RMS, frequency content
5. **Decision/Classification**: Thresholds, machine learning models

### 3.2 Microcontroller Selection

**Requirements**:
- Processing power: ARM Cortex-M4 or higher for complex algorithms
- Memory: 256 KB Flash, 64 KB RAM minimum
- Peripherals: ADC, I2C, SPI, UART, BLE or Wi-Fi
- Power efficiency: Sleep modes <10 μA

**Recommended MCUs**:
- nRF52840 (BLE, Cortex-M4, excellent power efficiency)
- ESP32 (Wi-Fi + BLE, dual-core, low cost)
- STM32L4 (ultra-low-power, extensive peripherals)

### 3.3 Machine Learning Integration

**Edge ML Frameworks**:
- TensorFlow Lite for Microcontrollers
- Edge Impulse
- MCUNet

**Typical Applications**:
- Gesture recognition from pressure patterns
- Gait classification from foot pressure
- Anomaly detection for health monitoring

**Model Optimization**:
- Quantization: 8-bit integer instead of 32-bit float (4× size reduction)
- Pruning: Remove unnecessary weights
- Knowledge distillation: Train smaller model to mimic larger one

---

## 4. Power Management

### 4.1 Power Budget

**Component Power Consumption** (typical):
- E-skin sensors: 0.1-1 mW per sensor (capacitive) to 1-10 mW (resistive with bias)
- Signal conditioning: 1-10 mW
- ADC: 1-5 mW
- MCU active: 10-50 mW
- MCU sleep: 0.01-0.1 mW
- BLE TX: 10-20 mW
- BLE sleep: 0.01 mW

**Total System** (64 sensors, BLE, continuous mode):
- Active: 50-150 mW
- Sleep: <1 mW

### 4.2 Battery Selection

**Li-Polymer**:
- Capacity: 100-1000 mAh typical for wearables
- Voltage: 3.7 V nominal
- Thin form factors available (1-5 mm)
- Requires protection circuit

**Coin Cells** (CR2032):
- Capacity: ~220 mAh
- Voltage: 3 V
- Low cost, compact
- Suitable for low-power, long-life applications

### 4.3 Power Optimization Strategies

- **Duty Cycling**: Sample sensors intermittently (e.g., 100 Hz for 100 ms every second)
- **Dynamic Sampling**: Increase rate only when activity detected
- **Efficient Wireless**: BLE connection interval 100-400 ms (balance latency and power)
- **Voltage Regulation**: Use efficient buck/boost converters (>90% efficiency)

---

## 5. Wireless Communication

### 5.1 Protocol Selection Matrix

| Application | Protocol | Range | Data Rate | Power | Latency |
|-------------|----------|-------|-----------|-------|---------|
| Prosthetics | BLE 5.0 | 10 m | 1 Mbps | Low | 10-50 ms |
| Health monitoring | BLE 5.0 | 10 m | 250 kbps | Very Low | 50-200 ms |
| Robotics | Wi-Fi 6 | 50 m | 10+ Mbps | Medium | 5-20 ms |
| Industrial | Zigbee | 100 m | 250 kbps | Low | 50-100 ms |

### 5.2 Data Packet Design

**Efficient Packet Structure**:
```
[Header: 2 bytes][Timestamp: 4 bytes][Sensor Data: N bytes][CRC: 2 bytes]
```

**Compression**:
- Delta encoding: Transmit change from previous value
- Run-length encoding: For sparse sensor arrays
- Lossy compression: Acceptable for some applications (e.g., discard low-order bits)

### 5.3 Wireless Security

**Encryption**: AES-128 for all medical data
**Authentication**: Pairing/bonding for BLE
**Key Management**: Secure storage in MCU, periodic rotation

---

## 6. Mechanical Integration

### 6.1 Substrate Attachment

**To Prosthetic Sockets**:
- Medical adhesive: 3M 1524 or equivalent
- Surface preparation: Clean with IPA, allow to dry
- Application: Press firmly, allow 1-hour cure before use
- Removal: Peel slowly at low angle, use adhesive remover if needed

**To Robots**:
- Mechanical fasteners with compliant layer
- Adhesive bonding with structural adhesive
- Over-molding (integrated during robot manufacturing)

### 6.2 Interconnects

**Flexible Flat Cables (FFC)**:
- Pitch: 0.5-1.0 mm
- Connectors: ZIF connectors on rigid PCB
- Routing: Avoid sharp bends (radius >10× thickness)

**Anisotropic Conductive Film (ACF)**:
- For direct e-skin to PCB connection
- Conductive in Z-direction only
- Application: Heat and pressure bonding

### 6.3 Strain Relief

- Serpentine routing of conductors
- Island-bridge architecture (rigid islands connected by stretchable bridges)
- Loose cable management (provide slack for stretching)

---

## 7. Firmware Development

### 7.1 Software Architecture

```
[Hardware Abstraction Layer]
        ↓
[Sensor Drivers]
        ↓
[Data Processing]
        ↓
[Application Logic]
        ↓
[Communication Stack]
```

### 7.2 Best Practices

- **Modular Code**: Separate sensor driver, processing, communication
- **Version Control**: Git with semantic versioning
- **Testing**: Unit tests for algorithms, integration tests for system
- **OTA Updates**: Plan for firmware updates in the field
- **Logging**: Detailed error logging for debugging

---

## 8. Calibration Procedures

### 8.1 Factory Calibration

1. **Zero-Point**: Record sensor output with no stimulation
2. **Span Calibration**: Apply known stimuli (weights, temperatures), record output
3. **Curve Fitting**: Determine calibration equation (linear, polynomial)
4. **Storage**: Save calibration coefficients to non-volatile memory
5. **Verification**: Apply test stimuli, verify accuracy

### 8.2 Field Calibration

- **User-Initiated**: Option for user to recalibrate (e.g., "Press firmly" for max pressure)
- **Automatic**: Periodic baseline adjustment
- **Reference Sensors**: Include dedicated reference sensor in known state

---

## 9. Testing and Validation

### 9.1 System-Level Tests

- **End-to-End**: Verify full data path from sensor to application
- **Performance**: Measure latency, accuracy, reliability
- **Stress Testing**: Extended operation, environmental extremes
- **User Acceptance**: Pilot testing with intended users

### 9.2 Failure Mode Analysis

- **FMEA** (Failure Modes and Effects Analysis): Identify potential failures, mitigation
- **Sensor Failure**: Detect and alert, use redundancy
- **Communication Loss**: Store data locally, retry transmission
- **Battery Depletion**: Low battery warning with adequate lead time

---

## 10. Regulatory Compliance

### 10.1 Medical Device Pathway (Class A)

- **Design Controls**: Per 21 CFR 820.30 (FDA) or ISO 13485
- **Risk Management**: ISO 14971
- **Clinical Validation**: Human factors testing, clinical trials
- **Documentation**: Design history file, device master record

### 10.2 Wireless Certification

- **FCC Part 15** (US): For unlicensed RF devices
- **CE Mark** (EU): RED (Radio Equipment Directive)
- **IC** (Canada): ISED certification

---

## Appendix A: Reference Designs

[Schematics and PCB layouts for common e-skin configurations]

## Appendix B: Code Examples

[Sample firmware for sensor readout, BLE communication, data processing]

---

**For Integration Support**: integration@wia-official.org

© 2025 SmileStory Inc. / WIA
