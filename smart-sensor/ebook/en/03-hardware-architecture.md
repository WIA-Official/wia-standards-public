# Chapter 3: Hardware Architecture and MCU Selection

## Building the Foundation for Intelligence at the Edge

The hardware architecture of a smart sensor system determines its capabilities, power efficiency, cost, and scalability. This chapter explores the critical decisions in designing smart sensor platforms, from MCU selection to peripheral integration.

---

## Smart Sensor Architecture Overview

### Typical Block Diagram

```
┌─────────────────────────────────────────────────────────┐
│                   Smart Sensor System                    │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌──────────┐      ┌──────────┐      ┌──────────────┐  │
│  │ Sensors  │─────▶│ Analog   │─────▶│ MCU Core     │  │
│  │ (MEMS,   │      │ Front-End│      │ ARM Cortex-M │  │
│  │  Optical,│      │ (ADC,    │      │ 80-200 MHz   │  │
│  │  Audio)  │      │  Op-Amp) │      │              │  │
│  └──────────┘      └──────────┘      └──────┬───────┘  │
│                                              │           │
│  ┌──────────┐      ┌──────────┐             │          │
│  │ Power    │      │ ML       │◀────────────┘          │
│  │ Mgmt     │      │ Accelerator              │          │
│  │ (LDO,    │      │ (Optional)│             │          │
│  │  DCDC,   │      └──────────┘             │          │
│  │  PMU)    │                                │          │
│  └──────────┘      ┌──────────┐             │          │
│                    │ Memory   │◀────────────┘          │
│  ┌──────────┐      │ (Flash,  │                        │
│  │ Wireless │      │  SRAM,   │                        │
│  │ (BLE,    │◀─────│  EEPROM) │                        │
│  │  LoRa,   │      └──────────┘                        │
│  │  WiFi)   │                                           │
│  └──────────┘                                           │
│                                                           │
│  ┌──────────────────────────────────────┐               │
│  │ Peripherals: I2C, SPI, UART, GPIO,   │               │
│  │ Timers, PWM, DMA, RTC, Watchdog      │               │
│  └──────────────────────────────────────┘               │
└─────────────────────────────────────────────────────────┘
```

### Key Components

**1. Sensor Elements**
- Physical transducers (accelerometer, temperature, microphone, etc.)
- Analog signal conditioning
- Multi-sensor integration

**2. Microcontroller Unit (MCU)**
- CPU core (ARM Cortex-M series, RISC-V)
- Memory (Flash, SRAM)
- Peripherals (timers, communication interfaces)

**3. ML Accelerator (Optional)**
- Dedicated hardware for neural network inference
- DSP extensions for signal processing
- Hardware floating-point unit (FPU)

**4. Power Management**
- Voltage regulation (LDO, DC-DC)
- Power mode control
- Energy harvesting interface

**5. Wireless Connectivity**
- BLE, LoRaWAN, NB-IoT, Wi-Fi
- Antenna and RF front-end
- Protocol stack

**6. Storage**
- Program memory (Flash)
- Data memory (SRAM)
- Non-volatile storage (EEPROM, external Flash)

---

## MCU Core Selection

### ARM Cortex-M Series Comparison

| Core | Clock | DMIPS | Power | FPU | DSP | Use Case |
|------|-------|-------|-------|-----|-----|----------|
| Cortex-M0+ | 48 MHz | 45 | Ultra-low | No | No | Basic sensing |
| Cortex-M3 | 100 MHz | 125 | Low | No | No | General IoT |
| Cortex-M4F | 180 MHz | 225 | Medium | Yes | Yes | ML inference |
| Cortex-M7F | 600 MHz | 2000 | Medium-high | Yes | Yes | Complex ML |
| Cortex-M33 | 200 MHz | 400 | Low-medium | Yes | Yes | Secure IoT |
| Cortex-M55 | 400 MHz | 800 | Medium | Yes | Yes + MVE | AI/ML focused |

### RISC-V Alternatives

**Advantages of RISC-V:**
- Open-source ISA (no licensing fees)
- Customizable instruction set extensions
- Growing ecosystem
- Vendor neutrality

**Popular RISC-V Cores for Smart Sensors:**

**SiFive E20**: Compact, low-power
- 2-stage pipeline
- 32-bit integer only
- 0.09 DMIPS/MHz
- Smallest silicon footprint

**SiFive E21**: Extended features
- 3-stage pipeline
- Optional multiply/divide
- 0.175 DMIPS/MHz
- Good for sensor hubs

**Nuclei N200**: Chinese RISC-V
- 2-stage pipeline with branch prediction
- Fast interrupt handling (ECLIC)
- DSP extensions available
- Optimized for IoT

**GreenWaves GAP8**: ML-focused
- 8 + 1 RISC-V cores (cluster + fabric controller)
- Hardware convolutional engine
- Ultra-low power (50 mW @ 175 MHz)
- Image and audio ML

### Decision Criteria

**Choose ARM Cortex-M when:**
- Mature ecosystem and tools are critical
- Wide vendor selection needed
- TrustZone security required (M33, M55)
- Proven reliability for safety applications

**Choose RISC-V when:**
- Cost optimization is paramount
- Custom instruction extensions needed
- Vendor lock-in is a concern
- Chinese market deployment

---

## Memory Architecture

### Flash Memory Sizing

**Code Storage:**
- Bootloader: 16-32 KB
- RTOS (FreeRTOS, Zephyr): 20-50 KB
- Application code: 50-200 KB
- ML model: 10-500 KB
- Libraries (math, DSP, crypto): 20-100 KB
- **Minimum: 128 KB, Recommended: 512 KB - 1 MB**

**Data Logging:**
- Sensor data buffer: 10-100 KB
- Event log: 10-50 KB
- Configuration: 1-10 KB

### SRAM Sizing

**Runtime Requirements:**
- Stack: 4-16 KB (depends on nesting depth)
- Heap: 10-100 KB (dynamic allocations, buffers)
- ML activation tensors: 5-200 KB (largest ML requirement)
- Sensor data buffers: 1-20 KB
- **Minimum: 32 KB, Recommended: 128-256 KB**

### Memory Bandwidth Considerations

**ML Inference Bottleneck:**

For a typical CNN layer:
- Weight memory access: Dominates computation time
- Activation tensor size: Determines SRAM requirement

**Example: MobileNet v2 Inverted Residual Block**

```
Input: 112×112×32 (401 KB)
Depthwise Conv 3×3: 32 filters
  Weights: 32×3×3 = 288 parameters (1.1 KB)
  MACs: 112×112×32×3×3 = 36M
Pointwise Conv 1×1: 32→16 channels
  Weights: 32×16 = 512 parameters (2 KB)
  MACs: 112×112×32×16 = 64M
Output: 112×112×16 (200 KB)

Peak memory: 401 KB (input) + 200 KB (output) = 601 KB
```

**Problem**: 601 KB exceeds typical MCU SRAM (64-256 KB)

**Solution: Layer-by-Layer Execution**
- Process image in tiles
- Reuse memory between layers
- Quantize to INT8 (4x reduction)
- **Reduced memory: 150 KB** (feasible!)

### External Memory Options

**When to add external memory:**
- ML models > 500 KB
- Data logging requirements
- Large lookup tables
- Audio/image buffering

**Technologies:**

**Serial Flash (SPI/QSPI):**
- Density: 1-128 MB
- Speed: 1-100 MB/s (quad SPI)
- Power: 5-15 mA active, < 1 µA sleep
- Cost: $0.20-$2
- Use: Code execution in place (XIP), data storage

**PSRAM (Pseudo-SRAM):**
- Density: 1-64 MB
- Speed: 20-80 MB/s
- Power: 3-10 mA active
- Cost: $0.50-$4
- Use: ML activation tensors, large buffers

**eMMC:**
- Density: 4-128 GB
- Speed: 50-200 MB/s
- Power: 50-150 mA active
- Cost: $3-$20
- Use: High-resolution image/video storage

---

## Power Management Architecture

### Power Domains

Smart sensors typically have multiple power domains:

**Always-On Domain:**
- RTC (Real-Time Clock)
- Backup SRAM
- Wakeup detection logic
- Power: 1-10 µA

**Sensor Domain:**
- Sensor analog front-end
- ADC
- Reference voltage
- Power: 10-500 µA (depends on sensor type)

**MCU Domain:**
- CPU core
- SRAM
- Peripherals
- Power: 0.5-50 mA (depends on clock speed, activity)

**Wireless Domain:**
- RF transceiver
- PA (Power Amplifier)
- Antenna switch
- Power: 5-150 mA (during TX/RX)

### Low-Power Modes

**Typical ARM Cortex-M Power Modes:**

| Mode | CPU | Peripherals | SRAM | Power | Wake-up Time |
|------|-----|-------------|------|-------|--------------|
| Run | On | On | Retained | 5-50 mA | N/A |
| Sleep | Off | On | Retained | 1-10 mA | < 1 µs |
| Deep Sleep | Off | Partial | Retained | 10-500 µA | 10-100 µs |
| Standby | Off | Off | Lost | 1-10 µA | 0.1-1 ms |
| Shutdown | Off | Off | Lost | 0.1-1 µA | 10-100 ms |

**Sleep Strategy Example:**

```c
// Sensor reads every 10 seconds, transmits every 10 minutes

void main_loop() {
    while (1) {
        // Wake up from deep sleep
        read_sensors();           // 100 ms @ 10 mA = 1 mJ

        if (sample_count % 60 == 0) {
            // Every 10 minutes
            process_ml_model();   // 50 ms @ 30 mA = 1.5 mJ
            transmit_ble();       // 5 ms @ 20 mA = 0.1 mJ
        }

        // Deep sleep for 10 seconds
        enter_deep_sleep(10000);  // 10s @ 0.05 mA = 0.5 mJ
    }
}

// Energy per cycle: 1 + 0.5 = 1.5 mJ (normal)
// Energy per cycle with TX: 1 + 1.5 + 0.1 + 0.5 = 3.1 mJ (every 60 cycles)
// Average energy per 10s: (59×1.5 + 3.1) / 60 = 1.52 mJ
// Average power: 1.52 mJ / 10s = 152 µW
// Battery life (CR2032, 220 mAh @ 3V): 220×3.6 / 0.152 = 5200 hours = 217 days
```

### Voltage Regulation

**LDO (Low Dropout Regulator):**
- Pros: Simple, low noise, low cost
- Cons: Poor efficiency (esp. with large Vin-Vout difference)
- Use: Low-current domains (< 50 mA), noise-sensitive analog

**Buck DC-DC Converter:**
- Pros: High efficiency (85-95%), wide input range
- Cons: Switching noise, larger footprint
- Use: Main power conversion, high current loads

**Hybrid Approach:**
- Buck converter for main MCU domain (high efficiency)
- LDO for analog front-end (low noise)
- Automatic switching based on load

**Example: STM32L4 Internal SMPS:**

```
Input: 3.3V (LiPo battery)
SMPS enabled → 1.8V internal
  MCU @ 80 MHz: 15 mA
  Efficiency: 90%
  Input current: 15 / 0.9 × (1.8/3.3) = 9 mA

SMPS disabled → 3.3V direct
  MCU @ 80 MHz: 20 mA
  Input current: 20 mA

Savings: 55%!
```

---

## Sensor Interface Design

### Analog Sensors

**Signal Chain:**
```
Sensor → Buffer Amp → Anti-Alias Filter → ADC → MCU
```

**ADC Selection Criteria:**

| Parameter | Requirement | Typical Value |
|-----------|-------------|---------------|
| Resolution | Sensor dynamic range | 12-16 bits |
| Sampling Rate | 2× max signal frequency | 1 kHz - 1 MHz |
| Input Range | Sensor output span | 0-3.3V, ±2.5V |
| Power | Battery life | 100 µA - 10 mA |
| Conversion Time | Latency budget | 1 µs - 1 ms |

**Oversampling for Increased Resolution:**

For an N-bit ADC, oversampling by 4^n samples gives n extra bits:

```
Effective bits = N + n
Oversample rate = 4^n

Example:
12-bit ADC → 14-bit effective resolution
Oversample: 4^2 = 16 samples
Noise reduction: √16 = 4×
```

### Digital Sensors

**I2C (Inter-Integrated Circuit):**
- Speed: Standard (100 kHz), Fast (400 kHz), Fast Plus (1 MHz)
- Topology: Multi-master, multi-slave bus
- Pins: 2 (SCL, SDA) + power
- Max devices: 112 (7-bit addressing)
- Use: Low-speed sensors (IMU, environmental, etc.)

**SPI (Serial Peripheral Interface):**
- Speed: Up to 50 MHz (depends on MCU)
- Topology: Master-slave, requires CS per slave
- Pins: 4 (MISO, MOSI, SCK, CS) + power
- Max devices: Limited by CS pins
- Use: High-speed sensors (ADC, IMU, optical)

**UART (Universal Asynchronous Receiver/Transmitter):**
- Speed: 9600 - 921600 baud (typical)
- Topology: Point-to-point
- Pins: 2 (TX, RX) + power
- Use: GPS modules, some environmental sensors

**I2S (Inter-IC Sound):**
- Speed: 128 - 512× sample rate
- Topology: Master-slave
- Pins: 3 (SCK, WS, SD) + power
- Use: Digital microphones, audio codecs

**PDM (Pulse Density Modulation):**
- Speed: 1-3.2 MHz
- Topology: Master (clock) - slave (data)
- Pins: 2 (CLK, DATA) + power
- Use: MEMS microphones (simpler than I2S)

### Multi-Sensor Integration

**Sensor Hub Approach:**

```c
// Pseudo-code for sensor hub
void sensor_hub_task() {
    // 100 Hz main loop
    while (1) {
        // Read IMU (accelerometer + gyroscope)
        imu_data = read_i2c_sensor(IMU_ADDR, IMU_REG, 12);
        accel = parse_accel(imu_data);
        gyro = parse_gyro(imu_data);

        // Read environmental sensor (temp, humidity, pressure)
        env_data = read_i2c_sensor(ENV_ADDR, ENV_REG, 8);
        temp = parse_temp(env_data);
        humidity = parse_humidity(env_data);
        pressure = parse_pressure(env_data);

        // Read magnetometer
        mag_data = read_i2c_sensor(MAG_ADDR, MAG_REG, 6);
        mag = parse_mag(mag_data);

        // Sensor fusion
        orientation = fusion_9dof(accel, gyro, mag);

        // Context detection
        context = ml_classify(accel, gyro, orientation);

        // Wake main MCU if context changed
        if (context != prev_context) {
            wake_main_mcu();
            send_context(context);
        }

        delay_ms(10); // 100 Hz
    }
}
```

**Benefits:**
- Main MCU can sleep most of the time
- Sensor hub handles continuous monitoring
- Interrupt-based wakeup only when needed
- Ultra-low system power

---

## Wireless Connectivity Selection

### Protocol Comparison

| Protocol | Range | Data Rate | Power (TX) | Power (Sleep) | Battery Life |
|----------|-------|-----------|------------|---------------|--------------|
| BLE 5.0 | 50-200m | 125k-2M bps | 10-20 mA | 1-5 µA | Months-Years |
| LoRaWAN | 2-15 km | 0.3-50 kbps | 20-120 mA | < 1 µA | Years |
| NB-IoT | 1-10 km | 20-250 kbps | 100-220 mA | 3-10 µA | Months-Year |
| Wi-Fi 4 | 50-100m | 1-150 Mbps | 100-350 mA | 0.5-5 mA | Days-Weeks |
| Zigbee | 10-100m | 250 kbps | 25-35 mA | 1-3 µA | Months-Years |
| Thread | 10-50m | 250 kbps | 25-35 mA | 2-5 µA | Months-Years |

### BLE 5.x Features for Smart Sensors

**Long Range Mode:**
- Coded PHY: 125 kbps or 500 kbps
- 4x range improvement (up to 400m outdoor)
- Cost: 2-8x energy per bit

**High Throughput Mode:**
- 2 Mbps PHY
- Faster data transfer = lower active time
- Better for burst transmissions

**Periodic Advertising:**
- Connectionless broadcast
- Ultra-low power (advertise every 1-10s)
- Good for beacon applications

**Extended Advertising:**
- 255-byte payloads (vs. 31 bytes in BLE 4.x)
- Include more sensor data per packet
- Reduce overhead

### LoRaWAN for Wide-Area Sensing

**Advantages:**
- Extreme range (15 km rural, 2 km urban)
- Ultra-low power (10-year battery life)
- Unlicensed spectrum (ISM bands)
- Public network infrastructure (TTN, Helium)

**Challenges:**
- Low data rate (max 50 kbps, typically 0.3-5 kbps)
- Duty cycle limitations (1% in EU, 36s/hour)
- Latency (class A: seconds, class C: near real-time)

**Use Cases:**
- Agricultural monitoring (soil moisture, weather)
- Smart cities (parking, waste management)
- Asset tracking (logistics, supply chain)
- Environmental monitoring (air quality, water level)

### Antenna Design

**Chip Antenna:**
- Pros: Small, integrated, no tuning
- Cons: Lower efficiency, narrow bandwidth
- Size: 2×3 mm typical
- Cost: $0.10-$0.50

**PCB Antenna:**
- Pros: No additional cost, customizable
- Cons: Requires RF expertise, PCB area
- Types: Inverted-F, meandered monopole, loop
- Design tools: AppCAD, AWR, Sonnet

**External Antenna:**
- Pros: Best performance, placement flexibility
- Cons: Connector cost, mechanical complexity
- Types: Whip, patch, helical
- Use: When range is critical, or metal enclosure

---

## Reference Designs

### Design 1: Ultra-Low-Power Environmental Monitor

**Specifications:**
- 10-year battery life (2× AA batteries)
- Temperature, humidity, pressure sensing
- LoRaWAN connectivity
- 1 measurement per 10 minutes

**Hardware:**
- MCU: STM32L072 (Cortex-M0+, ultra-low-power)
- Sensor: BME280 (Bosch environmental sensor)
- Radio: SX1276 (LoRa transceiver)
- Power: 2× AA Energizer Ultimate Lithium (3000 mAh @ 1.5V)

**Power Budget:**
- Deep sleep: 1 µA × 600 s = 600 µAs
- Sensor read: 3 mA × 0.1 s = 300 µAs
- MCU processing: 5 mA × 0.05 s = 250 µAs
- LoRa TX (SF7, 14 dBm): 40 mA × 0.1 s = 4000 µAs
- **Total per cycle: 5150 µAs**

**Battery Life:**
- Capacity: 3000 mAh = 10.8 million mAs = 10.8×10^9 µAs
- Cycles: 10.8×10^9 / 5150 = 2.1 million
- Years: 2.1M × 10min / (365.25×24×60) = **40 years**

(Limited by battery shelf life to ~10 years in practice)

### Design 2: Wearable Activity Tracker

**Specifications:**
- 7-day battery life (100 mAh LiPo)
- Continuous motion tracking
- Heart rate monitoring
- BLE connectivity
- OLED display

**Hardware:**
- MCU: nRF52840 (Cortex-M4F, BLE 5.0 integrated)
- IMU: LSM6DSO (ST, 6-axis with ML core)
- Heart rate: MAX30102 (Maxim, PPG sensor)
- Display: 0.96" OLED (128×64, I2C)
- Battery: 100 mAh LiPo (3.7V)

**Power Budget:**
- MCU sleep: 2 µA × 95% = 1.9 µA average
- MCU active: 7 mA × 5% = 350 µA average
- IMU (always-on): 200 µA continuous
- Heart rate (periodic): 1 mA × 10% = 100 µA average
- Display (occasional): 15 mA × 5% = 750 µA average
- BLE (advertising): 500 µA average
- **Total: 1.9 mA**

**Battery Life:**
- 100 mAh / 1.9 mA = **52.6 hours** (2.2 days)

**Optimization needed! Reduce display usage, lower BLE advertising rate:**
- Display: 15 mA × 1% = 150 µA (only when wrist raised)
- BLE: Lower to 200 µA (advertise every 2s instead of 1s)
- **New total: 1.0 mA**
- **Battery life: 100 hours = 4.2 days**

(Still short of 7-day target; would need larger battery or e-ink display)

---

## Review Questions

1. **MCU Selection Trade-offs**: Compare ARM Cortex-M4F and Cortex-M7F in terms of DMIPS, power consumption, and typical use cases. For a smart sensor requiring TinyML inference with 200 KB model size and 100 KB activation tensors, which core would you select and why?

2. **Memory Bottleneck Analysis**: A MobileNet v2 inverted residual block requires 601 KB peak memory (401 KB input + 200 KB output), but your target MCU has only 128 KB SRAM. Describe three specific techniques to reduce memory usage below 128 KB, including the layer-by-layer execution approach mentioned.

3. **Power Mode Optimization**: In the provided main_loop() example, calculate the average current consumption and verify the 217-day battery life claim. Given a CR2032 battery (220 mAh @ 3V), show your work for: (a) energy per 10-second cycle, (b) average power, (c) total battery life in days.

4. **Voltage Regulation Efficiency**: The STM32L4's internal SMPS achieves 55% power savings compared to direct LDO operation. Given the calculations shown (3.3V input, 1.8V internal, 15 mA @ 80 MHz), verify the efficiency percentage and explain when you would choose Buck DC-DC over LDO regulation.

5. **Wireless Protocol Selection**: Compare BLE 5.0 versus LoRaWAN for a smart agriculture deployment requiring 5 km range, 10-year battery life, and data transmission every 15 minutes. Calculate the energy per transmission for each protocol and recommend the optimal choice.

6. **Reference Design Analysis**: The ultra-low-power environmental monitor design claims 40-year theoretical battery life (limited to 10 years by shelf life). Break down the power budget showing all four components (deep sleep, sensor read, MCU processing, LoRa TX). What is the dominant energy consumer?

7. **Interface Protocol Matching**: Match each sensor type to its optimal digital interface: (a) High-speed 14-bit ADC, (b) MEMS IMU with 1 kHz update rate, (c) Digital MEMS microphone, (d) Environmental sensor reading every 100 ms. Options: I2C, SPI, I2S, PDM. Justify each selection.

## Key Takeaways

- **MCU Architecture Diversity**: ARM Cortex-M series spans from M0+ (45 DMIPS, ultra-low power) to M55 (800 DMIPS, AI-focused with MVE), while RISC-V alternatives like GreenWaves GAP8 offer **50 mW @ 175 MHz** with 8+1 cores for custom ML acceleration.

- **Memory Optimization Critical**: Typical TinyML models require **128-512 KB Flash** and **128-256 KB SRAM**, but clever techniques (INT8 quantization, layer-by-layer execution, tensor reuse) can reduce peak memory from 601 KB to **150 KB** for MobileNet v2.

- **Power Domain Hierarchy**: Smart sensors implement multi-domain power management: Always-On (1-10 µA), Sensor (10-500 µA), MCU (0.5-50 mA), and Wireless (5-150 mA), with Deep Sleep mode achieving **10-500 µA** system power while maintaining SRAM retention.

- **Voltage Regulation Impact**: STM32L4's internal SMPS achieves **55% power savings** (9 mA vs 20 mA input current @ 80 MHz) compared to direct LDO operation, demonstrating the critical role of efficient power conversion in battery-powered devices.

- **Wireless Trade-offs**: BLE 5.0 excels at **50-200m range with 10-20 mA TX power** (months battery life), while LoRaWAN extends to **2-15 km with 20-120 mA TX power** but ultra-low duty cycle enables **10-year battery life** on 2× AA cells.

- **Reference Design Validation**: The environmental monitor design achieves **40-year theoretical battery life** (10 years practical) with LoRaWAN by maintaining average power of **152 µW** (5150 µAs per 10-minute cycle), with LoRa TX consuming 78% of total energy budget.

- **Interface Protocol Selection**: I2C (up to 1 MHz) suits multi-sensor buses, SPI (up to 50 MHz) handles high-speed ADCs and IMUs, while I2S/PDM (1-3.2 MHz) optimally serve digital microphones with **lower pin count** and simplified MEMS integration.

---

**Next Chapter**: Embedded Machine Learning and TinyML frameworks for smart sensors.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
