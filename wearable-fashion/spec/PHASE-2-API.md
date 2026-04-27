# PHASE 2 — API

> Wearable-fashion interactive surface: LED and display
> technologies, sensors and interaction model, and the
> connectivity APIs that bridge the garment to phone /
> companion-device / cloud.

## 7. LED and Display Technologies

### 7.1 LED Types

#### 7.1.1 Individual RGB LEDs

**WS2812B (Neopixel)**
```
Specifications:
- Voltage: 5V ± 0.5V
- Current per LED: 60mA (max, white at full brightness)
- Current per color: 20mA
- Data protocol: Single-wire addressable
- Refresh rate: >400Hz
- Color depth: 24-bit (16.7M colors)
```

**APA102 (DotStar)**
```
Specifications:
- Voltage: 5V ± 0.5V
- Current per LED: 60mA (max)
- Data protocol: SPI (clock + data)
- Refresh rate: >10kHz
- Color depth: 24-bit + 5-bit brightness
```

#### 7.1.2 Fiber Optic LED

```
Configuration:
- Central LED source: 1-3W high-power LED
- Fiber count: 10-1000 strands
- Fiber diameter: 0.25-3mm
- Light transmission: >70% at 1m
- Flexibility: Excellent (bend radius <5mm)
```

#### 7.1.3 EL Wire (Electroluminescent)

```
Specifications:
- Voltage: 100-120VAC (via inverter)
- Frequency: 1000-3000Hz
- Brightness: 80-200 cd/m²
- Power: 0.2-0.5W per meter
- Diameter: 1.3-5mm
- Lifespan: 3000-8000 hours
```

### 7.2 LED Integration Methods

#### 7.2.1 Surface Mount

- Direct fabric mounting with conductive thread
- PCB strips sewn or bonded to fabric
- Flexibility: Limited (bend radius >20mm)
- Durability: High
- Density: Up to 144 LEDs/meter

#### 7.2.2 Embedded

- LEDs enclosed in waterproof sleeves
- Integrated into fabric weave
- Flexibility: Good (bend radius 10-15mm)
- Washability: Excellent (IP67+)
- Density: 30-60 LEDs/meter

#### 7.2.3 Woven Integration

- LEDs woven into fabric structure
- Conductive threads in warp/weft
- Flexibility: Excellent (fabric-like)
- Washability: Good (IP65)
- Density: 10-30 LEDs/100cm²

### 7.3 LED Patterns and Control

#### 7.3.1 Static Patterns

```cpp
// Example: Solid color
void setSolidColor(uint8_t r, uint8_t g, uint8_t b) {
    for(int i = 0; i < LED_COUNT; i++) {
        leds[i].setRGB(r, g, b);
    }
    FastLED.show();
}
```

#### 7.3.2 Dynamic Patterns

- Rainbow cycle
- Color chase
- Breathing effect
- Sparkle/twinkle
- Fire simulation
- Music reactive

#### 7.3.3 Interactive Patterns

- Touch-responsive
- Motion-activated
- Proximity-based
- Environmental (temperature, light)
- Biometric (heart rate, breathing)

### 7.4 Power Management for LEDs

#### 7.4.1 Brightness Control

```
Power Reduction = 1 - (Brightness / 255)²

Example:
50% brightness = 75% power reduction
25% brightness = 94% power reduction
```

#### 7.4.2 Duty Cycle Optimization

```
Average Power = Peak Power × Duty Cycle

Example:
10W peak, 30% duty cycle = 3W average
```

---


## 11. Sensors and Interaction

### 11.1 Input Sensors

#### 11.1.1 Touch Sensors

**Capacitive Touch**
```
Technology: Capacitance change detection
Sensitivity: 1-100pF range
Response time: <50ms
Sensing through: Up to 5mm fabric
Power consumption: 1-5mA
Implementation: Dedicated IC (e.g., TTP223) or MCU touch pins
```

**Resistive Touch**
```
Technology: Pressure-based resistance change
Activation force: 10-100g
Layers: Conductive fabric + separator + conductive fabric
Power: Only when pressed (<1μA standby)
Durability: >1 million presses
```

#### 11.1.2 Motion Sensors

**Accelerometer**
```
Specifications:
- Range: ±2g to ±16g
- Resolution: 10-16 bit
- Sample rate: 1-6400Hz
- Power: 0.1-3mA (active), <1μA (sleep)
- Interface: I2C or SPI
- Applications: Activity tracking, gesture recognition
```

**Gyroscope**
```
Specifications:
- Range: ±250 to ±2000°/s
- Resolution: 16 bit
- Noise: <0.01°/s/√Hz
- Power: 3-6mA
- Applications: Orientation, rotation detection
```

**IMU (Inertial Measurement Unit)**
```
Combined: Accelerometer + Gyroscope + Magnetometer
DOF: 9-axis (3 acc + 3 gyro + 3 mag)
Fusion: On-chip sensor fusion
Output: Quaternions, Euler angles
Update rate: Up to 1000Hz
```

#### 11.1.3 Biometric Sensors

**Heart Rate (PPG - Photoplethysmography)**
```
Technology: Optical blood flow detection
LEDs: Green (525nm) or IR (940nm)
Photodetector: Photodiode or phototransistor
Sampling: 25-100Hz
Accuracy: ±2 bpm (stationary), ±5 bpm (moving)
Power: 5-20mA continuous
Placement: Wrist, chest, finger
```

**Skin Temperature**
```
Sensor: NTC thermistor or digital (e.g., TMP117)
Range: 0-50°C
Accuracy: ±0.1-0.5°C
Response time: 1-30 seconds
Power: <1mA
```

**Galvanic Skin Response (GSR)**
```
Measurement: Skin conductance
Range: 0.1-20 μS
Application: Stress, emotion detection
Electrodes: Stainless steel or Ag/AgCl
Power: <1mA
```

### 11.2 Environmental Sensors

#### 11.2.1 Ambient Light

```
Sensor: Photodiode or ambient light IC (e.g., BH1750)
Range: 1-100000 lux
Resolution: 1 lux
Response time: <1 second
Power: 0.1-1mA
Applications: Automatic brightness adjustment
```

#### 11.2.2 Temperature/Humidity

```
Sensor: Combined T/H sensor (e.g., SHT31, DHT22)
Temperature:
  - Range: -40 to +125°C
  - Accuracy: ±0.2°C
Humidity:
  - Range: 0-100% RH
  - Accuracy: ±2%
Interface: I2C or 1-wire
Power: 0.3-1.5mA (measurement), <1μA (standby)
```

### 11.3 Haptic Feedback

#### 11.3.1 Vibration Motors

**ERM (Eccentric Rotating Mass)**
```
Specifications:
- Diameter: 6-12mm
- Voltage: 1.5-5V
- Current: 40-100mA
- Vibration frequency: 100-200Hz
- Response time: 50-100ms
- Cost: Low
```

**LRA (Linear Resonant Actuator)**
```
Specifications:
- Size: 7-25mm
- Voltage: 2-3.6V
- Current: 50-120mA
- Resonant frequency: 150-235Hz
- Response time: 10-30ms (faster than ERM)
- Efficiency: Higher than ERM
- Haptic effects: More precise
```

#### 11.3.2 Haptic Patterns

```cpp
// Example haptic patterns
void notificationBuzz() {
    vibrate(200, 100);  // 200ms on, 100ms off
    vibrate(200, 100);
}

void alertPattern() {
    vibrate(500, 200);  // Long buzz
    vibrate(100, 100);  // Short
    vibrate(100, 100);  // Short
}

void heartbeatPattern() {
    vibrate(50, 150);   // Beat
    vibrate(50, 600);   // Beat
}
```

### 11.4 User Interface

#### 11.4.1 Button Interfaces

- Single button: Mode cycling, power on/off
- Two buttons: Up/down, increase/decrease
- Three buttons: Mode, up, down
- Touch zones: Pattern selection, color choice

#### 11.4.2 Gesture Control

**Accelerometer-based Gestures**
```
- Tap: Quick acceleration spike
- Double tap: Two spikes within 500ms
- Shake: Sustained high-frequency movement
- Tilt: Orientation change
- Flip: 180° rotation
```

**Proximity Gestures**
```
- Hand wave: Reflectance change
- Hover: Distance <50mm
- Swipe: Direction of approach/retreat
```

---


## 12. Connectivity

### 12.1 Wireless Protocols

#### 12.1.1 Bluetooth Low Energy (BLE)

```
Standard: Bluetooth 5.0+
Range: 30-50m (open space)
Data rate: 1-2 Mbps
Power consumption:
  - TX: 8-15mA at 0dBm
  - RX: 8-12mA
  - Advertising: 0.5-2mA (periodic)
  - Connection idle: 0.5-3mA
  - Sleep: <1μA

Profile support:
- GATT (Generic Attribute Profile)
- Heart Rate Profile
- Battery Service
- Custom profiles

Advertising interval: 20ms-10.24s
Connection interval: 7.5ms-4s
```

#### 12.1.2 NFC (Near Field Communication)

```
Standard: ISO 14443
Range: <10cm (typically 0-5cm)
Frequency: 13.56 MHz
Data rate: 106-424 kbps
Power: <1mW (tag), 50-150mA (reader)

Applications:
- Pairing with smartphone
- Configuration transfer
- Contactless payment
- Identity verification
```

#### 12.1.3 WiFi

```
Standard: 802.11 b/g/n
Range: 50-100m
Data rate: 54-300 Mbps
Power consumption:
  - TX: 120-300mA
  - RX: 50-100mA
  - Sleep: 0.5-3mA

Use cases:
- High-bandwidth data transfer
- Cloud connectivity
- Firmware updates
- Video streaming (e.g., camera garments)

Limitations:
- High power consumption
- Less suitable for battery-powered wearables
```

### 12.2 Communication Protocols

#### 12.2.1 UART (Serial)

```
Baud rates: 9600-921600 bps
Pins: TX, RX, GND
Voltage: 3.3V or 5V logic
Use: Simple point-to-point communication
Applications: Debugging, simple peripherals
```

#### 12.2.2 I2C (Inter-Integrated Circuit)

```
Speed: 100 kHz (standard), 400 kHz (fast), 1 MHz (fast+)
Pins: SDA (data), SCL (clock), GND
Addressing: 7-bit or 10-bit
Multi-device: Yes (up to 127 devices)
Use: Sensors, displays, EEPROMs
```

#### 12.2.3 SPI (Serial Peripheral Interface)

```
Speed: Up to 10+ MHz
Pins: MOSI, MISO, SCK, CS, GND
Topology: Master-slave
Full-duplex: Yes
Use: High-speed communication (displays, LEDs, SD cards)
```

### 12.3 Data Management

#### 12.3.1 Data Types

```typescript
interface WearableData {
  timestamp: number;
  deviceId: string;

  // Sensor data
  heartRate?: number;        // bpm
  skinTemperature?: number;  // °C
  ambientLight?: number;     // lux
  motion?: {
    accel: [number, number, number];  // g
    gyro: [number, number, number];   // °/s
  };

  // Device status
  batteryLevel: number;      // 0-100%
  mode: string;
  ledPattern: string;
  brightness: number;        // 0-100%
}
```

#### 12.3.2 Data Transmission

**Efficient Data Encoding**
```
Binary format (vs JSON):
- Size reduction: 60-80%
- Parsing speed: 5-10x faster
- Power saving: Proportional to size reduction

Example:
JSON: {"heartRate":72,"temp":36.5,"battery":85}  // 47 bytes
Binary: [0x48, 0x00, 0xB5, 0x01, 0x55]          // 5 bytes
```

---


