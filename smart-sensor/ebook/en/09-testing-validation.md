# Chapter 9: Testing, Validation, and Compliance

## Ensuring Quality, Reliability, and Standards Compliance

Comprehensive testing ensures smart sensors meet performance, reliability, and safety requirements. This chapter covers testing methodologies, validation procedures, and compliance certification for smart sensor systems.

---

## Testing Pyramid for Smart Sensors

```
                 ┌──────────────┐
                 │   System     │  (End-to-end, Field)
                 │   Testing    │
                 └──────────────┘
              ┌─────────────────────┐
              │  Integration Testing │  (Sensor + MCU + Radio)
              └─────────────────────┘
          ┌─────────────────────────────┐
          │     Component Testing        │  (Individual sensors, modules)
          └─────────────────────────────┘
      ┌───────────────────────────────────────┐
      │         Unit Testing                  │  (Functions, algorithms)
      └───────────────────────────────────────┘
```

### Test Coverage Goals

- **Unit Tests**: 80%+ code coverage
- **Integration Tests**: All major interfaces
- **System Tests**: All use cases and failure modes
- **Field Testing**: Real-world validation

---

## Unit Testing

### Testing Embedded Firmware

**Framework**: Unity, CppUTest, Google Test (for C++)

**Example: Testing Sensor Calibration Function**

```cpp
#include "unity.h"
#include "sensor_calibration.h"

void setUp() {
    // Called before each test
    calibration_init();
}

void tearDown() {
    // Called after each test
    calibration_deinit();
}

void test_calibration_offset() {
    // Given: Raw sensor reading
    float raw_value = 123.45;

    // When: Apply calibration with known offset
    set_calibration_offset(10.0);
    float calibrated = apply_calibration(raw_value);

    // Then: Result should be raw + offset
    TEST_ASSERT_FLOAT_WITHIN(0.01, 133.45, calibrated);
}

void test_calibration_scale() {
    float raw_value = 100.0;

    set_calibration_scale(2.0);
    float calibrated = apply_calibration(raw_value);

    TEST_ASSERT_FLOAT_WITHIN(0.01, 200.0, calibrated);
}

void test_calibration_overflow() {
    // Test boundary condition
    float raw_value = MAX_SENSOR_VALUE + 10;

    float calibrated = apply_calibration(raw_value);

    // Should clamp to max
    TEST_ASSERT_EQUAL_FLOAT(MAX_SENSOR_VALUE, calibrated);
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_calibration_offset);
    RUN_TEST(test_calibration_scale);
    RUN_TEST(test_calibration_overflow);

    return UNITY_END();
}
```

### Testing ML Models

**Accuracy, Precision, Recall:**

```python
import numpy as np
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score

def test_ml_model_accuracy():
    # Load test dataset
    X_test, y_test = load_test_data()

    # Run inference
    y_pred = model.predict(X_test)

    # Calculate metrics
    accuracy = accuracy_score(y_test, y_pred)
    precision = precision_score(y_test, y_pred, average='weighted')
    recall = recall_score(y_test, y_pred, average='weighted')
    f1 = f1_score(y_test, y_pred, average='weighted')

    # Assert minimum requirements
    assert accuracy >= 0.95, f"Accuracy {accuracy:.2%} below threshold"
    assert precision >= 0.93, f"Precision {precision:.2%} below threshold"
    assert recall >= 0.93, f"Recall {recall:.2%} below threshold"
    assert f1 >= 0.93, f"F1 score {f1:.2%} below threshold"

def test_ml_model_inference_time():
    # Measure inference latency on target hardware
    import time

    input_data = generate_test_input()

    times = []
    for _ in range(100):
        start = time.time()
        _ = model.predict(input_data)
        end = time.time()
        times.append((end - start) * 1000)  # Convert to ms

    avg_time = np.mean(times)
    max_time = np.max(times)

    # Assert latency requirements
    assert avg_time < 50, f"Average inference time {avg_time:.1f}ms too slow"
    assert max_time < 100, f"Max inference time {max_time:.1f}ms too slow"
```

### Mock Testing for Hardware

**Separate hardware dependencies for testing:**

```cpp
// sensor_interface.h (abstraction)
typedef struct {
    int (*init)(void);
    int (*read)(float* value);
    int (*sleep)(void);
} SensorInterface;

// sensor_real.c (actual hardware)
int sensor_real_init() {
    // Configure I2C, initialize sensor chip
    return i2c_sensor_init(SENSOR_ADDR);
}

int sensor_real_read(float* value) {
    uint16_t raw;
    i2c_read_register(SENSOR_ADDR, DATA_REG, &raw);
    *value = raw * 0.01;  // Convert to physical units
    return 0;
}

SensorInterface sensor_real = {
    .init = sensor_real_init,
    .read = sensor_real_read,
    .sleep = sensor_real_sleep
};

// sensor_mock.c (for testing)
static float mock_value = 25.0;

int sensor_mock_init() {
    return 0;  // Always succeeds
}

int sensor_mock_read(float* value) {
    *value = mock_value;
    return 0;
}

void sensor_mock_set_value(float value) {
    mock_value = value;
}

SensorInterface sensor_mock = {
    .init = sensor_mock_init,
    .read = sensor_mock_read,
    .sleep = sensor_mock_sleep
};

// Application code (uses abstraction)
void test_temperature_threshold_alert() {
    SensorInterface* sensor = &sensor_mock;
    sensor->init();

    // Set mock temperature to trigger alert
    sensor_mock_set_value(35.0);

    float temp;
    sensor->read(&temp);

    bool alert = check_temperature_alert(temp, 30.0);

    TEST_ASSERT_TRUE(alert);
}
```

---

## Integration Testing

### Sensor + MCU Integration

**Test sensor communication, data integrity, timing:**

```cpp
void test_sensor_i2c_communication() {
    // Initialize I2C bus
    i2c_init(I2C1, 400000);  // 400 kHz

    // Initialize sensor
    int ret = sensor_init();
    TEST_ASSERT_EQUAL(0, ret);

    // Verify WHO_AM_I register
    uint8_t who_am_i;
    i2c_read_register(SENSOR_ADDR, WHO_AM_I_REG, &who_am_i);
    TEST_ASSERT_EQUAL(EXPECTED_WHO_AM_I, who_am_i);

    // Configure sensor
    sensor_configure(SENSOR_MODE_NORMAL, SENSOR_ODR_100HZ);

    // Read multiple samples
    for (int i = 0; i < 100; i++) {
        float value;
        ret = sensor_read(&value);
        TEST_ASSERT_EQUAL(0, ret);
        TEST_ASSERT_TRUE(value >= -50 && value <= 150);  // Sanity check

        delay_ms(10);
    }
}

void test_sensor_data_ready_interrupt() {
    // Configure sensor to generate interrupt on data ready
    sensor_enable_interrupt(SENSOR_INT_DATA_READY);

    // Set up GPIO interrupt
    volatile bool irq_fired = false;
    gpio_set_interrupt(SENSOR_INT_PIN, IRQ_RISING, []() {
        irq_fired = true;
    });

    // Trigger sensor measurement
    sensor_trigger_measurement();

    // Wait for interrupt (with timeout)
    uint32_t start = millis();
    while (!irq_fired && (millis() - start) < 1000) {
        __WFI();  // Sleep until interrupt
    }

    TEST_ASSERT_TRUE(irq_fired);
}
```

### Wireless Communication Testing

**BLE Connection Test:**

```cpp
void test_ble_connection() {
    // Start BLE advertising
    ble_advertising_start();

    // Wait for central to connect (use test fixture/app)
    uint32_t timeout = 30000;  // 30s
    uint32_t start = millis();
    while (!ble_is_connected() && (millis() - start) < timeout) {
        delay_ms(100);
    }

    TEST_ASSERT_TRUE(ble_is_connected());

    // Test characteristic write
    uint8_t test_data[] = {0x01, 0x02, 0x03};
    ble_gatts_set_value(test_char_handle, test_data, sizeof(test_data));

    // Verify central received data (requires test app confirmation)
    // ...

    // Test notification
    ble_gatts_notify(test_char_handle);

    // Disconnect
    ble_gap_disconnect();
    delay_ms(100);
    TEST_ASSERT_FALSE(ble_is_connected());
}
```

**LoRaWAN Over-the-Air Test:**

```cpp
void test_lorawan_join_and_transmit() {
    // Join network
    int ret = lorawan_join_otaa(DEV_EUI, APP_EUI, APP_KEY);
    TEST_ASSERT_EQUAL(0, ret);

    // Wait for join accept (up to 30s)
    uint32_t timeout = 30000;
    uint32_t start = millis();
    while (!lorawan_is_joined() && (millis() - start) < timeout) {
        lorawan_process();
        delay_ms(100);
    }

    TEST_ASSERT_TRUE(lorawan_is_joined());

    // Transmit test message
    uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    ret = lorawan_send(1, payload, sizeof(payload), false);
    TEST_ASSERT_EQUAL(0, ret);

    // Wait for transmission complete
    while (lorawan_is_busy()) {
        lorawan_process();
        delay_ms(10);
    }

    // Verify message received by network server (requires backend check)
    // ...
}
```

---

## System Testing

### Environmental Testing

**Temperature Cycling:**

```
Test Procedure:
1. Place device in environmental chamber
2. Cycle temperature: -20°C → 60°C → -20°C
3. Ramp rate: 5°C/min
4. Hold at each extreme: 2 hours
5. Number of cycles: 10

Measurements:
- Sensor accuracy at each temperature
- Power consumption variation
- Wireless connectivity (range, throughput)
- Battery voltage

Pass Criteria:
- Sensor accuracy: ±2% over full temperature range
- No functional failures
- Wireless connectivity maintained
```

**Humidity Testing:**

```
Conditions:
- 85°C, 85% RH (relative humidity)
- Duration: 1000 hours (IPC-TM-650)

Inspection:
- Visual inspection for corrosion
- Functional testing after exposure
- Insulation resistance measurement
```

**Vibration Testing:**

```
Standard: IEC 60068-2-6
Profile: Random vibration
Frequency: 10-2000 Hz
PSD: 0.04 g²/Hz
Duration: 1 hour per axis (X, Y, Z)

Pass Criteria:
- No mechanical failure
- Sensor accuracy maintained: ±5%
- Wireless connectivity functional
```

### Power Consumption Validation

**Measure actual vs. specification:**

```cpp
void test_power_consumption_sleep() {
    // Enter deep sleep mode
    enter_deep_sleep();

    // Measure current (use power profiler or DMM)
    // Expected: < 10 µA

    // Automated testing with power profiler
    power_profiler_start();
    delay_ms(10000);  // Measure for 10 seconds
    float avg_current = power_profiler_get_average();
    power_profiler_stop();

    TEST_ASSERT_LESS_THAN(10e-6, avg_current);  // < 10 µA
}

void test_power_consumption_active() {
    // Active mode: sensor sampling + ML inference
    power_profiler_start();

    for (int i = 0; i < 100; i++) {
        read_sensors();
        run_ml_inference();
        delay_ms(100);
    }

    float avg_current = power_profiler_get_average();
    float energy = power_profiler_get_total_energy();

    power_profiler_stop();

    // Expected: < 15 mA average
    TEST_ASSERT_LESS_THAN(15e-3, avg_current);

    // Energy budget check
    float expected_energy = 15e-3 * 10;  // 15 mA × 10s
    TEST_ASSERT_LESS_THAN(expected_energy, energy);
}
```

### Battery Life Validation

**Accelerated testing:**

```
Method: Duty-cycle simulation at elevated temperature

Normal operation:
- Sample every 60s
- Transmit every 10 minutes
- Deep sleep between

Accelerated test (10× faster):
- Sample every 6s
- Transmit every 60s
- Elevated temperature: 50°C

Duration:
- Target battery life: 3 years
- Accelerated test: 4 months (10× faster + temp acceleration)

Measurement:
- Battery voltage over time
- Capacity consumed
- Extrapolate to normal conditions
```

### Fail-Safe and Recovery Testing

**Watchdog Recovery:**

```cpp
void test_watchdog_recovery() {
    // Configure watchdog
    watchdog_init(5000);  // 5s timeout

    // Simulate firmware hang
    while (1) {
        // Infinite loop, no watchdog refresh
    }

    // Device should reset via watchdog
    // (Test framework verifies reset occurred and device recovered)
}
```

**Low-Battery Handling:**

```cpp
void test_low_battery_shutdown() {
    // Simulate low battery voltage
    battery_simulator_set_voltage(2.0);  // Below 2.2V threshold

    // Application should detect and shut down gracefully
    battery_monitor_task();

    // Verify:
    TEST_ASSERT_TRUE(is_low_battery_flag_set());
    TEST_ASSERT_TRUE(flash_write_complete());  // Data saved
    TEST_ASSERT_TRUE(is_shutdown_initiated());
}
```

**Communication Failure Recovery:**

```cpp
void test_lorawan_reconnection() {
    // Join network
    lorawan_join_otaa();
    while (!lorawan_is_joined()) lorawan_process();

    // Simulate network loss (e.g., move out of range)
    lorawan_simulate_link_loss();

    // Application should detect and rejoin
    for (int i = 0; i < 5; i++) {
        lorawan_send(data, size, false);
        lorawan_process();

        if (!lorawan_is_joined()) {
            // Should trigger rejoin
            lorawan_process_rejoin();
        }
    }

    // Verify rejoined
    TEST_ASSERT_TRUE(lorawan_is_joined());
}
```

---

## Field Testing

### Real-World Deployment Trials

**Phase 1: Alpha Testing (Internal)**
- 10-50 devices
- Controlled environment
- Frequent data collection
- Rapid iteration

**Phase 2: Beta Testing (Limited External)**
- 100-500 devices
- Real user environments
- Weekly data review
- Firmware updates as needed

**Phase 3: Production Pilot**
- 1,000-10,000 devices
- Full production configuration
- Monthly reviews
- Limited updates (only critical bugs)

### Field Data Collection

**Telemetry to collect:**

```cpp
typedef struct {
    uint32_t uptime;
    uint16_t reset_count;
    uint8_t reset_reason;  // Watchdog, brownout, software, etc.
    uint16_t battery_mv;
    int8_t rssi;  // Signal strength
    uint8_t tx_failures;
    uint8_t rx_timeouts;
    uint16_t ml_inferences_total;
    uint16_t ml_errors;
    float sensor_min;
    float sensor_max;
    float sensor_avg;
    uint8_t sensor_errors;
} Telemetry;

void collect_telemetry(Telemetry* telem) {
    telem->uptime = millis() / 1000;
    telem->reset_count = get_reset_count();
    telem->reset_reason = get_reset_reason();
    telem->battery_mv = read_battery_voltage();
    telem->rssi = get_last_rssi();
    // ... (collect all metrics)
}

void send_telemetry() {
    Telemetry telem;
    collect_telemetry(&telem);

    // Send to cloud for analysis
    transmit(&telem, sizeof(telem));
}
```

**Analysis:**
- Reset rate: Should be < 1 per month
- Battery drain: Validate power model
- Signal strength: Identify coverage gaps
- ML accuracy: Field vs. lab comparison
- Sensor failures: Identify defective units

---

## Compliance and Certification

### Regulatory Testing

**FCC (USA) - Radio Emissions:**

**FCC Part 15 (Unlicensed devices):**
- Conducted emissions
- Radiated emissions
- Bandwidth

**Test Setup:**
- Anechoic chamber
- Spectrum analyzer
- Turntable (for radiated tests)

**Pass Criteria:**
- Fundamental emission within allocated band
- Spurious emissions < -60 dBc
- Out-of-band emissions meet mask

**FCC Part 22/24 (Cellular - NB-IoT):**
- Requires certification by cellular carrier
- PTCRB or GCF certification
- Specific to network operator

**CE (Europe):**

**RED (Radio Equipment Directive):**
- Health and safety
- Electromagnetic compatibility (EMC)
- Efficient use of spectrum

**EN 300 328 (2.4 GHz ISM band):**
- For BLE, WiFi, Zigbee
- Adaptive frequency hopping
- Power spectral density limits

**EN 300 220 (Sub-1 GHz):**
- For LoRaWAN, proprietary Sub-GHz
- Duty cycle limits
- EIRP (Effective Isotropic Radiated Power) limits

**MIC (Japan):**
- ARIB STD-T66 (BLE, WiFi)
- ARIB STD-T108 (LoRaWAN)
- Technical conformity certification

### Safety Standards

**IEC 61508 (Functional Safety):**

**SIL (Safety Integrity Level):**
- SIL 1: Low risk
- SIL 2: Medium risk
- SIL 3: High risk (medical, automotive)
- SIL 4: Very high risk (rare for sensors)

**Requirements:**
- Systematic capability (design process)
- Random hardware failures (MTBF analysis)
- Safety manual

**ISO 26262 (Automotive):**

**ASIL (Automotive Safety Integrity Level):**
- ASIL A: Lowest
- ASIL B, C: Medium
- ASIL D: Highest (critical safety functions)

**Sensors for ADAS typically require ASIL B or higher.**

**UL 2900 (Cybersecurity):**
- Software vulnerability assessment
- Penetration testing
- Security lifecycle management

### Environmental Compliance

**RoHS (Restriction of Hazardous Substances):**
- Limits on lead, mercury, cadmium, hexavalent chromium, PBB, PBDE
- < 0.1% (1000 ppm) for most substances
- < 0.01% (100 ppm) for cadmium

**REACH (Registration, Evaluation, Authorization of Chemicals):**
- Disclosure of SVHCs (Substances of Very High Concern)
- Material declaration for supply chain

**WEEE (Waste Electrical and Electronic Equipment):**
- Recycling requirements
- Marking requirements (crossed-out wheelie bin symbol)
- Producer responsibility

---

## Continuous Testing and Monitoring

### CI/CD for Firmware

**Automated Build and Test:**

```yaml
# .gitlab-ci.yml example
stages:
  - build
  - test
  - deploy

build_firmware:
  stage: build
  script:
    - cd firmware
    - make clean
    - make all
  artifacts:
    paths:
      - firmware/build/firmware.bin
      - firmware/build/firmware.elf

unit_tests:
  stage: test
  script:
    - cd firmware/tests
    - make test
    - ./run_tests --xml-output=test_results.xml
  artifacts:
    reports:
      junit: firmware/tests/test_results.xml

integration_tests:
  stage: test
  script:
    - python3 test_harness.py --device /dev/ttyUSB0
  tags:
    - hardware-test-rig

deploy_to_test_fleet:
  stage: deploy
  script:
    - python3 ota_deploy.py --target beta_fleet --firmware firmware.bin
  only:
    - develop
```

### Fleet Monitoring

**Cloud-based monitoring dashboard:**

```
Metrics to track:
- Device online/offline status
- Battery levels (histogram across fleet)
- Firmware versions deployed
- Error rates (sensor, communication, ML)
- Network performance (RSSI, latency, packet loss)
- Geographic distribution
- Daily active devices

Alerts:
- Sudden increase in error rate
- Multiple devices offline in same region (network issue?)
- Battery drain faster than model predicts
- Security events (failed authentication attempts)
```

---

## WIA-SEMI-015 Compliance Checklist

**Hardware:**
☐ MCU meets minimum performance (ARM Cortex-M4 or equivalent)
☐ Memory: ≥ 128 KB SRAM, ≥ 512 KB Flash
☐ Secure boot implemented
☐ Hardware RNG available
☐ Low-power modes: < 50 µA deep sleep

**Sensors:**
☐ At least one digital sensor with interrupt capability
☐ Sensor accuracy documented and validated
☐ Temperature compensation implemented

**ML:**
☐ On-device inference capability
☐ Model size: < 500 KB
☐ Inference latency: < 100 ms
☐ INT8 quantization supported

**Connectivity:**
☐ At least one wireless protocol (BLE, LoRaWAN, WiFi, NB-IoT)
☐ Secure communication (TLS/DTLS or equivalent)
☐ OTA update capability

**Power:**
☐ Battery life ≥ 1 year (for battery-powered devices)
☐ Power consumption measured and documented
☐ Low-battery detection and graceful shutdown

**Security:**
☐ Encrypted communication
☐ Secure key storage
☐ Firmware signature verification
☐ Rollback protection

**Software:**
☐ RTOS or bare-metal with documented architecture
☐ Error handling and recovery mechanisms
☐ Watchdog timer enabled
☐ Unit test coverage ≥ 60%

**Testing:**
☐ Environmental testing performed (-20°C to +60°C)
☐ EMI/EMC compliance tested
☐ Field trials with ≥ 100 devices for ≥ 1 month
☐ Regression testing automated

**Documentation:**
☐ Hardware design files available
☐ Software architecture documented
☐ API reference complete
☐ User manual provided
☐ Safety and compliance certificates

---

## Conclusion

Comprehensive testing and validation ensure smart sensors meet their promises: accuracy, reliability, security, and longevity. From unit tests to field trials, each layer builds confidence in the system.

The WIA-SEMI-015 standard provides a framework for developing high-quality smart sensors that benefit users while respecting privacy and security. As the technology continues to evolve, ongoing testing and monitoring remain essential.

**弘益人間** - May these smart sensors truly benefit all humanity.

---

## Review Questions

1. **Test Coverage Metrics**: The testing pyramid recommends 80%+ unit test coverage, all major interfaces for integration tests, and all use cases for system tests. For a smart sensor project with 10,000 lines of code, 20 interfaces, and 50 use cases, estimate the total number of test cases needed at each level. Which level provides the best ROI?

2. **ML Model Validation**: The keyword spotting model requires ≥95% accuracy, ≥93% precision/recall, <50 ms average inference, <100 ms max inference. If your model achieves 94.8% accuracy, what actions would you take? Explain the difference between precision and recall in the context of false wake-word detections.

3. **Environmental Testing Standards**: Calculate the total test duration for: temperature cycling (10 cycles, 2-hour holds at -20°C and 60°C, 5°C/min ramp), humidity testing (1000 hours @ 85°C/85% RH), and vibration testing (3 hours for X/Y/Z axes). What is the minimum calendar time needed?

4. **Battery Life Validation**: A 3-year target battery life cannot be tested in real-time. Describe the 10× accelerated testing methodology (6s sampling vs. 60s, elevated temperature). If accelerated test shows 10% capacity loss after 4 months, project the real-world battery life. Does it meet the 3-year target?

5. **Power Profiling Analysis**: Your device spec is <10 µA sleep, <15 mA active. Measurements show: sleep 12 µA, active 14 mA (both within spec individually), but battery life is 50% below prediction. What additional measurements would you take to identify the issue? Consider wake-up transitions, unexpected wake events, and duty cycle accuracy.

6. **Field Telemetry Interpretation**: Fleet monitoring shows: reset rate 5 per month (vs. target <1), RSSI averaging -95 dBm (vs. design -85 dBm), and ML error rate 3% (vs. lab 0.5%). Diagnose the likely root causes for each metric. What actions would you take?

7. **Compliance Cost-Benefit**: FCC/CE certification costs $20K-50K, safety certification (IEC 61508 SIL 2) costs $100K-300K, and cybersecurity testing (UL 2900) costs $50K-150K. For a product with 10,000-unit production volume at $50 retail price, calculate the per-unit compliance cost. Is this justified for a consumer vs. industrial/medical product?

## Key Takeaways

- **Testing Pyramid**: Unit tests (80%+ code coverage) form the base, integration tests cover all major interfaces, system tests validate all use cases, and field testing (100+ devices, 1+ month) provides real-world validation. **Automated CI/CD** enables continuous regression testing with <5-minute build-test-deploy cycles.

- **ML Model Validation**: Require **≥95% accuracy, ≥93% precision/recall**, measured on held-out test sets. Inference latency testing (100 iterations) must show **<50 ms average, <100 ms worst-case** on target hardware (Cortex-M4 @ 80 MHz), with explicit handling of edge cases and adversarial examples.

- **Environmental Testing**: IEC 60068 compliance requires **temperature cycling** (-20°C to +60°C, 10 cycles, 2-hour holds), **humidity exposure** (85°C/85% RH, 1000 hours), and **vibration testing** (10-2000 Hz, 0.04 g²/Hz PSD, 3 hours per axis), validating ±2% sensor accuracy and functional reliability across operating conditions.

- **Accelerated Battery Life**: 10× accelerated testing (elevated temperature + 10× duty cycle) enables **3-year validation in 4 months**. Power profiling must measure all states (sleep, active, wireless TX) with **1 µA resolution** to identify discrepancies between model predictions and actual consumption.

- **Regulatory Compliance**: FCC Part 15 (conducted/radiated emissions), CE-RED (health/safety/EMC/spectrum), MIC Japan (ARIB standards) certify wireless operation. Safety standards (IEC 61508 SIL 1-4, ISO 26262 ASIL A-D) and cybersecurity (UL 2900, PSA Certified) add **$50K-$500K certification cost** depending on criticality.

- **Fleet Monitoring**: Cloud dashboard tracks **reset rate (<1/month target)**, battery histogram, error rates (sensor/comm/ML), network performance (RSSI/latency/packet loss), and geographic distribution. **Automated alerts** detect sudden error rate increases, regional outages, and security events (failed auth attempts).

- **WIA-SEMI-015 Compliance**: Mandates ARM Cortex-M4+ (or equivalent), **≥128 KB SRAM / ≥512 KB Flash**, <50 µA deep sleep, on-device ML (<500 KB model, <100 ms inference), wireless connectivity with **TLS/DTLS encryption**, OTA updates, **≥60% unit test coverage**, and environmental testing (-20°C to +60°C).

---

**End of WIA-SEMI-015 Smart Sensor Standard Ebook**

For more resources:
- Simulator: https://wia-standards.org/semi-015/simulator
- Specification: https://wia-standards.org/semi-015/spec
- API Documentation: https://wia-standards.org/semi-015/api
- Ebook: https://wiabooks.store/tag/wia-smart-sensor

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
