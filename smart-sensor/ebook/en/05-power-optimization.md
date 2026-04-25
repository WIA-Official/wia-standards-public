# Chapter 5: Power Optimization and Energy Harvesting

## Achieving Years of Battery Life

Power consumption is often the defining constraint for smart sensors. This chapter explores comprehensive strategies for minimizing power consumption and enabling battery-free operation through energy harvesting.

---

## Understanding Power Consumption

### Power vs Energy

**Power (P)** = Rate of energy consumption (Watts = Joules/second)
**Energy (E)** = Total consumption over time (Joules = Watt-seconds)

For battery-powered devices:
```
Battery Life = Battery Capacity (Wh) / Average Power (W)
```

**Example:**
- Battery: CR2032 (220 mAh @ 3V = 0.66 Wh)
- Average power: 100 µW = 0.0001 W
- Battery life: 0.66 / 0.0001 = **6,600 hours = 275 days**

### Power States in Smart Sensors

Smart sensors cycle through multiple power states:

**Active State:**
- All systems operational
- CPU running at full speed
- Sensors sampling
- Power: 5-50 mA

**Idle State:**
- CPU halted, peripherals active
- Quick wake-up (< 1 µs)
- Power: 1-10 mA

**Sleep State:**
- Most peripherals off
- RTC and wakeup sources active
- Wake-up: 10-100 µs
- Power: 10-500 µA

**Deep Sleep State:**
- Only essential circuits active
- SRAM retained
- Wake-up: 0.1-10 ms
- Power: 1-50 µA

**Shutdown State:**
- Everything off except wakeup pin
- SRAM lost
- Wake-up: 10-100 ms
- Power: 0.1-5 µA

---

## Duty Cycling: The Foundation of Low Power

### Basic Duty Cycling

**Concept**: Active only when necessary, sleep the rest of the time.

```
Duty Cycle = Active Time / Total Time

Average Power = (Active Power × Duty Cycle) + (Sleep Power × (1 - Duty Cycle))
```

**Example: Environmental Sensor**

Measure temperature every 60 seconds:
- Active: 10 ms @ 10 mA
- Sleep: 59.99 s @ 5 µA

```
Duty Cycle = 10 ms / 60,000 ms = 0.0167%
Avg Power = (10 mA × 0.000167) + (5 µA × 0.999833)
          = 1.67 µA + 5 µA = 6.67 µA
```

**Battery Life (CR2032):**
```
220 mAh / 6.67 µA = 32,985 hours = 3.76 years
```

### Dynamic Duty Cycling

**Adaptive sampling** based on conditions:

```cpp
int get_sample_interval() {
    float temperature = read_temperature();

    // Sample more frequently if temperature changing rapidly
    if (abs(temperature - prev_temperature) > 2.0) {
        return 10;  // 10 seconds (high activity)
    } else if (abs(temperature - prev_temperature) > 0.5) {
        return 60;  // 1 minute (moderate activity)
    } else {
        return 300; // 5 minutes (stable)
    }
}
```

**Benefit**: Responsiveness when needed, low power when stable.

**Power Savings Example:**
- Fixed 60s interval: 6.67 µA average
- Adaptive (90% stable, 10% active):
  - 90% of time: 300s interval → 1.67 µA
  - 10% of time: 10s interval → 11.67 µA
  - Average: 0.9×1.67 + 0.1×11.67 = **2.67 µA**

**60% power reduction!**

### Event-Driven Architecture

Instead of periodic sampling, wake on events:

**Interrupt Sources:**
- Motion detection (accelerometer threshold)
- Button press
- Audio activity (voice detection)
- External signal (GPIO)
- Timer (RTC alarm)

**Example: Wearable with Motion Wake**

```cpp
void setup() {
    // Configure accelerometer for motion detection
    accel.setWakeThreshold(50);  // 50 mg threshold
    accel.enableInterrupt(INT1);

    // Configure MCU to wake on accelerometer interrupt
    attachInterrupt(INT1, motion_detected, RISING);

    // Enter deep sleep
    enter_deep_sleep();
}

void motion_detected() {
    // Wake up and process
    read_full_sensor_data();
    run_ml_inference();

    // If no motion for 10s, go back to sleep
    if (motion_idle_time > 10000) {
        enter_deep_sleep();
    }
}
```

**Power Comparison:**
- Periodic sampling (10 Hz): 500 µA
- Event-driven (sleep until motion): **15 µA**

**33× power reduction!**

---

## Sensor Power Optimization

### Low-Power Sensor Selection

**Key Metrics:**
- **Active current**: During measurement
- **Sleep current**: Between measurements
- **Measurement time**: Faster = less energy
- **Supply voltage range**: Lower voltage = less power

**Example: Temperature Sensors**

| Sensor | Active | Sleep | Time | Energy/Sample |
|--------|--------|-------|------|---------------|
| TMP117 | 135 µA | 150 nA | 15.5 ms | 2.09 µJ |
| Si7051 | 210 µA | 60 nA | 10 ms | 2.1 µJ |
| LM75 | 1 mA | 3.5 µA | 100 ms | 100 µJ |
| DHT22 | 1.5 mA | 15 µA | 2000 ms | 3000 µJ |

**TMP117 is 1400× more efficient than DHT22!**

### Sensor Power Modes

Many sensors have multiple power modes:

**Bosch BMA400 Accelerometer:**
- Normal mode: 14 µA (full featured)
- Low power mode: 3.5 µA (100 Hz sampling)
- Ultra-low power: 1 µA (25 Hz)
- Sleep: 0.8 µA

**Strategy**: Match sensor mode to application needs.

### Sensor Duty Cycling

Turn sensors off when not needed:

```cpp
void read_environmental_data() {
    // Power on sensor
    sensor_power_enable();
    delay_ms(10);  // Sensor startup time

    // Read measurement
    float temp = sensor.readTemperature();
    float humidity = sensor.readHumidity();

    // Power off sensor
    sensor_power_disable();

    // Process data
    process_data(temp, humidity);
}
```

**Power Savings:**
- Always on: 500 µA
- Duty cycled (on 1s every 60s): **10 µA**

**50× reduction!**

### FIFO Buffering

Sensors with FIFOs allow burst reading:

```cpp
// Instead of reading accelerometer 100 times/second:
// BAD: Wake MCU 100× per second
for (int i = 0; i < 100; i++) {
    wake_mcu();
    read_accel();  // 1 sample
    sleep_mcu(10); // 10 ms
}

// GOOD: Wake MCU 1× per second
wake_mcu();
read_accel_fifo(samples, 100);  // 100 samples
process_samples(samples, 100);
sleep_mcu(1000);  // 1 second
```

**MCU wake overhead savings:**
- Frequent wake: 100× × 1 ms = 100 ms active/second
- FIFO burst: 1× × 10 ms = 10 ms active/second

**10× MCU power reduction!**

---

## MCU Power Optimization

### Clock Speed Optimization

**Lower frequency = lower power** (usually cubic relationship):

```
Power ≈ C × V² × f

C = capacitance
V = voltage
f = frequency
```

**Example: STM32L476**

| Frequency | Voltage | Current | Power |
|-----------|---------|---------|-------|
| 80 MHz | 1.2V | 100 µA/MHz | 8 mA |
| 40 MHz | 1.2V | 70 µA/MHz | 2.8 mA |
| 16 MHz | 1.2V | 50 µA/MHz | 0.8 mA |
| 4 MHz | 1.0V | 30 µA/MHz | 0.12 mA |

**Strategy**: Run as slow as possible while meeting latency requirements.

**Dynamic Frequency Scaling:**

```cpp
void process_sensor_data() {
    // Quick task: Run at low speed
    set_clock(4_MHz);
    read_sensors();
    set_clock(80_MHz);  // Boost for ML

    // ML inference
    run_ml_model();

    // Back to low speed
    set_clock(4_MHz);
}
```

### Peripheral Management

**Disable unused peripherals:**

```cpp
void enter_low_power() {
    // Disable all unused peripherals
    RCC->AHB1ENR = 0;  // Disable AHB1 peripherals
    RCC->AHB2ENR = 0;  // Disable AHB2 peripherals
    RCC->APB1ENR = RTC_EN;  // Only RTC enabled
    RCC->APB2ENR = 0;  // Disable APB2 peripherals

    // Configure GPIO for lowest leakage
    for (int i = 0; i < 16; i++) {
        GPIOA->MODER &= ~(3 << (i * 2));  // Analog mode
        GPIOB->MODER &= ~(3 << (i * 2));
        GPIOC->MODER &= ~(3 << (i * 2));
    }
}
```

**Savings:**
- All peripherals enabled: 500 µA
- Minimal peripherals: **50 µA**

**10× reduction!**

### DMA for Zero-CPU Data Transfer

Use DMA to move data without CPU involvement:

```cpp
// Without DMA: CPU moves every byte
for (int i = 0; i < 1024; i++) {
    buffer[i] = ADC->DR;  // CPU active
}

// With DMA: CPU sleeps while DMA works
DMA_Config(ADC_DR, buffer, 1024);
DMA_Start();
enter_sleep();  // CPU sleeps
// DMA interrupt wakes CPU when done
```

**Power Savings:**
- CPU polling ADC: 10 mA × 10 ms = 100 µJ
- DMA + CPU sleep: 2 µA × 10 ms = 20 nJ

**5000× reduction!**

### Cache and Memory Optimization

**Enable instruction/data cache:**

```cpp
// Cortex-M7 with cache
SCB_EnableICache();
SCB_EnableDCache();

// Speedup: 2-4×
// Power: 30-50% reduction (less memory access)
```

**Keep critical code in SRAM:**

```cpp
// Place frequently executed code in SRAM (faster, less power)
__attribute__((section(".ramfunc")))
void critical_sensor_loop() {
    // This runs from SRAM, not Flash
    // 2× faster, 40% less power
}
```

---

## Communication Power Optimization

### Protocol Selection

**Energy per bit comparison:**

| Protocol | Energy/bit | Range | Comments |
|----------|-----------|-------|----------|
| BLE 5.0 | 5-10 nJ | 50-200m | Best for short bursts |
| LoRaWAN | 200-500 nJ | 2-15 km | Best for long range |
| NB-IoT | 100-300 nJ | 1-10 km | Cellular infrastructure |
| Wi-Fi | 50-100 nJ | 50-100m | High throughput |

**Strategy**: Match protocol to application needs.

### Transmission Optimization

**Reduce data transmitted:**

```cpp
// BAD: Send raw sensor data (100 samples × 12 bytes = 1200 bytes)
transmit(raw_samples, 1200);

// GOOD: Send ML inference result (1 byte)
int gesture = ml_classify(raw_samples);
transmit(&gesture, 1);

// 1200× data reduction!
```

**Batch transmissions:**

```cpp
// BAD: Send every sample (100× connection overhead)
for (int i = 0; i < 100; i++) {
    ble_connect();
    ble_send(sample[i]);
    ble_disconnect();
}

// GOOD: Batch samples (1× connection overhead)
ble_connect();
for (int i = 0; i < 100; i++) {
    ble_send(sample[i]);
}
ble_disconnect();

// 100× overhead reduction!
```

### BLE Power Optimization

**Connection parameters:**

```cpp
// High power: 7.5ms interval, 0 latency
// Power: 500 µA average

// Low power: 1000ms interval, 4 latency
// Power: 15 µA average

ble_set_connection_params(
    .min_interval = 800,  // 800 × 1.25ms = 1s
    .max_interval = 800,
    .latency = 4,  // Skip 4 intervals
    .timeout = 4000
);
```

**Advertising optimization:**

```cpp
// High frequency advertising (discoverable)
ble_set_advertising_interval(100);  // 100ms, ~200 µA

// Low power advertising
ble_set_advertising_interval(2000); // 2s, ~20 µA

// 10× power reduction
```

### LoRaWAN Optimization

**Spreading factor selection:**

| SF | Bitrate | Range | Airtime (51 bytes) | Energy |
|----|---------|-------|-------------------|--------|
| SF7 | 5.5 kbps | 2 km | 56 ms | 5.6 mJ |
| SF9 | 1.76 kbps | 5 km | 205 ms | 20.5 mJ |
| SF12 | 250 bps | 15 km | 1.4 s | 140 mJ |

**Strategy**: Use lowest SF that provides reliable link.

**ADR (Adaptive Data Rate):**

```cpp
// Let network optimize SF and TX power
lorawan_enable_adr(true);

// Network gradually reduces SF as link improves
// Energy savings: 5-10×
```

---

## Energy Harvesting

### Solar Energy

**Indoor Solar (Fluorescent/LED):**
- Power density: 10-100 µW/cm²
- Panel size: 2-10 cm²
- Available power: 20-1000 µW

**Outdoor Solar:**
- Power density: 10-100 mW/cm² (sunny)
- Panel size: 1-5 cm²
- Available power: 10-500 mW

**Example: Indoor Solar Sensor**

```
Solar panel: 5 cm² @ 50 µW/cm² = 250 µW
Sensor power: 20 µW average
Battery charging: 230 µW
Battery: Supercapacitor, 1 F @ 3.3V

Energy storage: ½ × 1 × 3.3² = 5.4 J
Runtime (no light): 5.4 J / 20 µW = 270,000s = 75 hours

Perpetual operation in lit areas!
```

**Design considerations:**
- MPPT (Maximum Power Point Tracking) for efficiency
- Supercapacitor or thin-film battery
- Ultra-low-power design (< 100 µW average)

### Vibration Energy

**Piezoelectric harvesters:**
- Power: 10-100 µW (from machinery vibration)
- Frequency: 50-200 Hz
- Voltage: 1-10V (needs regulation)

**Example: Industrial Vibration Sensor**

```
Harvester: 50 µW average
Sensor: 30 µW average (with duty cycling)
Storage: Supercapacitor, 0.1 F

Cold start: Charge for 1000s to reach 3V
  Energy: ∫ 50 µW dt = 50 mJ
  Capacitor: ½ × 0.1 × 3² = 0.45 J (450 mJ)

Perpetual operation once started!
```

### Thermal Energy Harvesting

**Thermoelectric generators (TEG):**
- Power: 1-10 mW (from 10°C temperature gradient)
- Voltage: 0.1-1V
- Efficiency: 2-5%

**Applications:**
- Body heat (skin to ambient: 5-10°C)
- Industrial pipes (hot fluid to ambient: 20-100°C)
- HVAC ducts

**Example: Body-Powered Wearable**

```
TEG: 5°C gradient × 2 mW/°C = 10 mW
Wearable: 2 mW average
Excess: 8 mW for battery charging

Battery: 100 mAh LiPo
Charging time: 100 mAh / 8 mA = 12.5 hours
Full charge overnight!
```

### RF Energy Harvesting

**Principle**: Capture ambient RF energy from TV/radio/cellular signals.

**Power levels:**
- Close to source (1m): 1-10 µW
- Medium distance (10m): 0.1-1 µW
- Far distance (100m): 0.01-0.1 µW

**Applications:**
- RFID tags
- Wireless sensor tags (Wiliot, Powercast)
- Implantable medical devices

**Example: RF-Powered Sensor Tag**

```
RF harvester: 2 µW @ 10m from WiFi router
Sensor: 1 µW average (1% duty cycle, 100 µW active)
Net charging: 1 µW

Supercapacitor: 0.01 F
Charge time (0 to 3V): 0.5 × 0.01 × 3² / 1 µW = 45,000s = 12.5 hours

Cold start: 12.5 hours
Perpetual operation after initial charge!
```

---

## Ultra-Low-Power Design Patterns

### Pattern 1: Wake-on-Pattern

Dedicated low-power pattern detector wakes main system:

```
Audio Example:
┌─────────────┐      ┌──────────────┐      ┌─────────┐
│ Microphone  │─────▶│ Wake Word    │─────▶│ Main    │
│ (always-on) │      │ Detector     │      │ CPU     │
│ 100 µA      │      │ (TinyML)     │      │ (sleep) │
└─────────────┘      │ 200 µA       │      └─────────┘
                     └──────────────┘
                            │
                      [Wake if "Hey Device" detected]
                            │
                            ▼
                     ┌──────────────┐
                     │ Main CPU     │
                     │ (active)     │
                     │ 20 mA        │
                     └──────────────┘

Average power:
- Wake word: 300 µA (always)
- Main CPU: 20 mA × 0.1% (active 0.1% of time)
- Total: 320 µA

Without wake-on-pattern (always-on main CPU):
- 20 mA (2000× higher!)
```

### Pattern 2: Hierarchical Power Domains

Multiple MCUs with increasing capability and power:

```
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│ Sensor MCU   │─▶│ Edge MCU     │─▶│ Main MCU     │
│ (Cortex-M0+) │  │ (Cortex-M4)  │  │ (Cortex-A7)  │
│ 5 µA sleep   │  │ 50 µA sleep  │  │ 5 mA sleep   │
│ 500 µA active│  │ 10 mA active │  │ 200 mA active│
└──────────────┘  └──────────────┘  └──────────────┘

Sensor MCU: Always-on sensor monitoring
  → Wakes Edge MCU if interesting pattern

Edge MCU: Local ML processing
  → Wakes Main MCU if classification threshold met

Main MCU: Complex processing, connectivity
  → Only active when absolutely necessary

Average power: 50 µA (vs. 5 mA if Main MCU always-on)
100× reduction!
```

### Pattern 3: Asynchronous Operation

Decouple sensor sampling from processing:

```cpp
// Producer: Fast interrupt-driven sampling
void ADC_IRQHandler() {
    buffer[write_index++] = ADC->DR;
    if (write_index >= BUFFER_SIZE) {
        write_index = 0;
        buffer_full_flag = true;
    }
}

// Consumer: Process when convenient
void main_loop() {
    while (1) {
        if (buffer_full_flag) {
            process_buffer(buffer);
            buffer_full_flag = false;
        }

        // Sleep until next buffer ready
        enter_sleep_until_interrupt();
    }
}

// CPU sleeps while waiting for buffer to fill
// Only wakes for processing (10% duty cycle)
```

---

## Power Measurement and Profiling

### Tools

**Hardware:**
- Multimeter (basic average current)
- Oscilloscope + current probe (transients)
- Power analyzer (Keysight N6705, Otii Arc)
- Energy profiler (Nordic nRF Power Profiler, STM32CubeMonitor-Power)

**Software:**
- RTOS power profiling (FreeRTOS configGENERATE_RUN_TIME_STATS)
- MCU internal current sensing (some STM32, nRF52)
- Simulator (ARM Fast Models, Renode)

### Profiling Methodology

**1. Baseline measurement:**
```
Measure sleep current with all peripherals off
Target: < 10 µA for battery-operated
```

**2. Per-peripheral profiling:**
```
Enable one peripheral at a time
Measure increase in current
Identify power hogs
```

**3. Dynamic profiling:**
```
Run full application
Capture current vs. time
Identify unexpected wake-ups, long active periods
```

**4. Integration:**
```
Calculate weighted average based on duty cycles
Predict battery life
Iterate to meet targets
```

### Example: Power Profiling Results

```
Device: Environmental Sensor
Target: 3-year battery life on CR2032 (220 mAh)
Budget: 220 mAh / 26,280 hours = 8.37 µA

Measured:
- Sleep (95%): 3 µA × 0.95 = 2.85 µA
- Sensor read (4%): 500 µA × 0.04 = 20 µA
- LoRa TX (1%): 100 mA × 0.01 = 1000 µA
- Total: 1022.85 µA

Prediction: 220 mAh / 1022 µA = 215 hours = 9 days

Problem: 122× over budget!

Optimization:
- Reduce TX rate: 1% → 0.01% (every 10 min → every 16 hours)
  New TX: 100 mA × 0.0001 = 10 µA
- Sensor duty cycle: 4% → 1% (longer sleep between samples)
  New sensor: 500 µA × 0.01 = 5 µA
- Optimize sleep: 3 µA → 1 µA (disable more peripherals)

New total: 1 + 5 + 10 = 16 µA
Battery life: 220 mAh / 16 µA = 13,750 hours = 1.57 years

Close! Further optimization (sleep to 0.5 µA, sensor to 3 µA):
Total: 0.5 + 3 + 10 = 13.5 µA
Battery life: 220 mAh / 13.5 µA = 16,296 hours = 1.86 years

Still short. Increase battery to 2× AA (3000 mAh):
3000 mAh / 13.5 µA = 222,222 hours = 25 years ✓
```

---

## Review Questions

1. **Duty Cycle Calculation**: An environmental sensor operates with 10 ms active time @ 10 mA and 59.99 s sleep @ 5 µA. Calculate: (a) duty cycle percentage, (b) average power consumption, (c) CR2032 battery life (220 mAh). Show all calculations and verify the 3.76-year claim.

2. **Dynamic Duty Cycling Benefits**: The adaptive sampling example achieved 60% power reduction (from 6.67 µA to 2.67 µA) by adjusting intervals based on temperature change rate. Explain the strategy used (300s stable, 60s moderate, 10s active) and calculate the power breakdown for the 90% stable / 10% active scenario.

3. **Sensor Selection Impact**: Compare TMP117 versus DHT22 temperature sensors in terms of energy per sample. Given the provided data (TMP117: 135 µA for 15.5 ms; DHT22: 1.5 mA for 2000 ms), calculate energy consumption and explain why TMP117 is **1400× more efficient**.

4. **MCU Clock Frequency Optimization**: Using the STM32L476 data (80 MHz @ 8 mA, 40 MHz @ 2.8 mA, 16 MHz @ 0.8 mA, 4 MHz @ 0.12 mA), demonstrate that the Power ≈ C × V² × f relationship doesn't hold linear. Calculate the actual power scaling factor between 80 MHz and 4 MHz.

5. **Communication Protocol Energy**: Compare BLE versus LoRaWAN for an application requiring 1 transmission per 10 minutes. Given: BLE (20 mA for 5 ms), LoRaWAN SF7 (40 mA for 56 ms). Calculate average power for each protocol including 5 µA sleep current. Which is more efficient and by what factor?

6. **Energy Harvesting Feasibility**: An indoor solar sensor has a 5 cm² panel yielding 50 µW/cm² and requires 20 µW average power. Calculate: (a) available power, (b) charging power, (c) energy stored in 1 F supercapacitor at 3.3V, (d) runtime without light. Verify the 75-hour claim.

7. **Power Profiling Case Study**: The environmental sensor example initially measured 1022.85 µA (vs. 8.37 µA target). Trace through all three optimization iterations (TX reduction, sensor duty cycle, sleep optimization) and calculate the final power budget after switching to 2× AA batteries (3000 mAh).

## Key Takeaways

- **Duty Cycling Fundamentals**: Basic duty cycling with 10 ms active @ 10 mA every 60 seconds achieves **6.67 µA average power** (99.98% sleep time), enabling **3.76-year battery life** on CR2032, demonstrating that aggressive sleep strategies are the foundation of ultra-low-power design.

- **Event-Driven Architecture**: Motion-wake architecture reduces power from **500 µA (periodic 10 Hz sampling) to 15 µA (sleep until motion)**, achieving **33× power reduction** by eliminating continuous polling in favor of interrupt-driven wake-up from accelerometer thresholds.

- **Sensor Efficiency Variance**: Temperature sensor selection dramatically impacts energy: TMP117 consumes **2.09 µJ per sample** versus DHT22's **3000 µJ**, representing **1400× efficiency difference**, highlighting the critical importance of low-power sensor selection in battery-operated designs.

- **FIFO Buffering Benefits**: Accelerometer FIFO batching enables reading 100 samples in one MCU wake (10 ms active) versus 100 individual wakes (100 ms active), delivering **10× MCU power reduction** by amortizing wake-up overhead across multiple samples.

- **Communication Optimization**: Data reduction through edge ML processing (1200 bytes raw accelerometer → 1 byte classification) achieves **1200× bandwidth savings**, while batching 10 samples per transmission provides **10× overhead reduction** in MQTT publish operations.

- **Indoor Solar Viability**: A 5 cm² solar panel @ 50 µW/cm² (250 µW total) can sustain a 20 µW sensor with **230 µW battery charging**, storing **5.4 J** in 1 F supercapacitor for **75-hour dark operation**, demonstrating perpetual battery-free operation in lit environments.

- **Systematic Power Debugging**: The environmental sensor power profiling case study reduced consumption from **1022.85 µA** (9-day battery life) to **13.5 µA** (25-year life with 2× AA) through iterative optimization: LoRa TX duty cycle (1% → 0.01%), sensor sampling (4% → 1%), and sleep current (3 µA → 0.5 µA).

---

**Next Chapter**: Sensor fusion and multi-modal sensing architectures.

---

© 2025 SmileStory Inc. / WIA
弘익人間 (Hongik Ingan) · Benefit All Humanity
