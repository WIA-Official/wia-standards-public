# Chapter 5: Environmental Sensors

## Introduction to Environmental Sensing

Environmental sensors monitor physical and chemical parameters of the surrounding environment, including temperature, humidity, pressure, air quality, and light. These sensors are critical for applications ranging from weather stations and HVAC systems to IoT devices and smart cities.

### Temperature Sensors

#### Temperature Measurement Principles

**Thermal Expansion:**
- Bimetallic strips, liquid thermometers
- Analog mechanical systems
- Limited use in modern electronics

**Electrical Resistance Change:**
- Metal oxides: Resistance increases with temperature (PTC)
- Semiconductors: Resistance decreases with temperature (NTC)
- Precision metals: Linear resistance change (RTD)

**Thermoelectric Effect:**
- Seebeck effect: Junction of dissimilar metals
- Thermocouples: Generate voltage proportional to temperature difference
- Wide temperature range (-200°C to +1800°C)

**Semiconductor Bandgap:**
- PN junction forward voltage varies with temperature
- Integrated circuit temperature sensors
- Linear, easy to interface digitally

#### Thermistor Technology

**NTC (Negative Temperature Coefficient):**

**Characteristic Equation:**
```
R(T) = R₀ × exp[B × (1/T - 1/T₀)]

Where:
R(T) = Resistance at temperature T (Kelvin)
R₀ = Resistance at reference temperature T₀
B = Material constant (2000-5000 K)
T = Absolute temperature (Kelvin)
```

**Typical Values:**
- R₀ = 10 kΩ @ 25°C
- B = 3950 K
- Accuracy: ±0.1°C to ±0.5°C
- Range: -40°C to +125°C

**Applications:**
- Battery temperature monitoring
- HVAC control
- Automotive coolant temperature
- Consumer electronics thermal management

**PTC (Positive Temperature Coefficient):**
- Self-limiting heaters
- Overcurrent protection (resettable fuses)
- Not typically used for precision measurement

#### RTD (Resistance Temperature Detector)

**Platinum RTD (Pt100, Pt1000):**

**Temperature Coefficient:**
```
R(T) = R₀ × [1 + α×(T - T₀) + β×(T - T₀)²]

Standard: IEC 60751
α = 0.00385 Ω/Ω/°C
Pt100: R₀ = 100Ω @ 0°C
Pt1000: R₀ = 1000Ω @ 0°C
```

**Classes:**
- Class AA: ±(0.1 + 0.0017×|T|)°C
- Class A: ±(0.15 + 0.002×|T|)°C
- Class B: ±(0.3 + 0.005×|T|)°C

**Advantages:**
- High accuracy and stability
- Linear response
- Wide temperature range (-200°C to +850°C)
- Industry standards (interchangeable)

**Disadvantages:**
- More expensive than thermistors
- Requires precision current source
- Self-heating effects
- Slower response time

**Applications:**
- Industrial process control
- Laboratory calibration standards
- Food processing
- Pharmaceutical cold chain

#### Thermocouple Sensors

**Common Types:**

| Type | Materials | Range (°C) | Sensitivity (μV/°C) | Applications |
|------|-----------|------------|---------------------|--------------|
| K | Chromel-Alumel | -200 to +1350 | 41 | General purpose, industrial |
| J | Iron-Constantan | -40 to +750 | 52 | Inert atmospheres |
| T | Copper-Constantan | -200 to +350 | 43 | Low temperature, high accuracy |
| E | Chromel-Constantan | -200 to +900 | 61 | Highest EMF output |
| N | Nicrosil-Nisil | -200 to +1300 | 39 | High stability, oxidation resistant |
| R/S | Pt-Rh/Pt | 0 to +1600 | 10 | High temperature, ceramics |
| B | Pt-Rh/Pt-Rh | +200 to +1800 | 10 | Highest temperature |

**Cold Junction Compensation:**
- Thermocouples measure temperature difference
- Reference junction must be known
- Semiconductor cold junction on PCB
- Accuracy: ±0.5°C to ±2°C

**Signal Conditioning:**
- Microvolt-level signals (10-60 μV/°C)
- High-gain low-noise amplifiers
- Linearization (polynomial or lookup table)
- Digital output: MAX31855, MAX31856

**Applications:**
- Furnaces and kilns
- Engine exhaust temperature
- Molten metal processing
- Welding equipment

#### Integrated Circuit Temperature Sensors

**Analog Output (LM35, TMP36):**
```
V_out = 10 mV/°C (LM35)
V_out = 750 mV @ 25°C, slope = 10 mV/°C (TMP36)

Accuracy: ±0.5°C to ±2°C
Range: -40°C to +125°C
Power: <50 μA
```

**Digital Output (DS18B20, TMP117):**

**DS18B20 (1-Wire):**
- Resolution: 9-12 bits (0.5°C to 0.0625°C)
- Accuracy: ±0.5°C (-10°C to +85°C)
- 1-Wire bus: Multiple sensors on single wire
- Parasite power mode
- Unique 64-bit serial number

**TMP117 (I2C, High Precision):**
- Resolution: 16-bit (0.0078°C)
- Accuracy: ±0.1°C (0°C to +50°C), ±0.2°C (-40°C to +100°C)
- Low power: 3.5 μA active, 150 nA shutdown
- NIST-traceable calibration
- Application: Medical, precision instrumentation

**On-Chip Temperature Sensors:**
- Every microcontroller, FPGA, SoC has integrated sensor
- Accuracy: ±3°C to ±5°C (uncalibrated)
- Purpose: Thermal management, overheat protection
- Examples: ARM Cortex internal DTS, Intel Digital Thermal Sensor

### Humidity Sensors

#### Relative Humidity Definition

```
RH = (P_v / P_vs) × 100%

Where:
P_v = Partial pressure of water vapor
P_vs = Saturation vapor pressure at given temperature
```

**Dew Point:**
Temperature at which air becomes saturated (RH = 100%).

**Absolute Humidity:**
Mass of water vapor per unit volume of air (g/m³).

#### Capacitive Humidity Sensors

**Operating Principle:**
- Dielectric constant of polymer changes with moisture
- Capacitance varies with relative humidity
- Sensor structure: Polymer dielectric between electrodes

**Humidity-Capacitance Relationship:**
```
C(RH) = C₀ × [1 + k × RH]

Typical:
C₀ = 180 pF @ 0% RH
C = 200 pF @ 100% RH
Δ = +11% for 100% RH change
```

**Sensirion SHT4x:**
- Accuracy: ±1.8% RH (typical)
- Resolution: 0.01% RH
- Response time: 8 seconds (τ₆₃)
- Temperature sensor integrated (±0.2°C)
- I2C interface
- Power: 0.4 μA average @ 1 measurement/s
- Size: 1.5 × 1.5 × 0.55 mm³

**Bosch BME280:**
- Combined sensor: Humidity + Temperature + Pressure
- RH accuracy: ±3%
- Operating range: 0-100% RH
- Temperature: -40°C to +85°C
- Pressure: 300-1100 hPa
- I2C/SPI interface
- Widely used in IoT, weather stations

**TE Connectivity HTU21D:**
- Accuracy: ±2% RH
- Resolution: 12-bit (0.04% RH)
- Low power: 2.4 μA active, 0.02 μA sleep
- I2C interface

**Calibration:**
- Factory calibration with salt solutions
  - MgCl₂: 33% RH @ 20°C
  - NaCl: 75% RH @ 20°C
  - K₂SO₄: 97% RH @ 20°C
- Hysteresis: ±1-2% RH
- Long-term drift: <0.5% RH/year

#### Resistive Humidity Sensors

**Principle:**
- Ionic conduction through hygroscopic material
- Resistance decreases with increasing humidity
- Cheaper than capacitive, less accurate

**Materials:**
- Metal oxides, conductive polymers
- Salt-based ceramics

**Disadvantages:**
- Non-linear response
- Sensitive to contamination
- Higher power consumption
- Limited lifespan

#### Thermal Conductivity Humidity Sensors

**Principle:**
- Thermal conductivity of air changes with moisture
- Heated thermistor, measure cooling rate
- Absolute humidity measurement

**Advantages:**
- Not affected by condensation
- Measures absolute humidity directly

**Disadvantages:**
- High power (heater)
- Slow response
- Specialized applications only

### Pressure Sensors (Barometric)

#### Atmospheric Pressure Sensing

**Purpose:**
- Weather forecasting
- Altitude measurement
- Vertical positioning (floor detection in buildings)

**Pressure-Altitude Relationship:**
```
h = 44330 × [1 - (P / P₀)^0.1903]

Where:
h = Altitude (meters)
P = Measured pressure (Pa)
P₀ = Sea-level pressure (101325 Pa)

Pressure gradient: ~12 Pa/m near sea level
100 Pa = ~8.3 meters altitude change
```

#### MEMS Barometric Sensors

**Bosch BMP390:**
- Pressure range: 300-1250 hPa
- Absolute accuracy: ±0.5 hPa (±4.1 m altitude)
- Relative accuracy: ±0.06 hPa (±0.5 m)
- Resolution: 0.016 Pa (0.13 mm!)
- RMS noise: 0.03 Pa (ultra-low)
- Temperature coefficient: ±0.75 Pa/K
- Current: 3.2 μA @ 1 Hz
- Size: 2.0 × 2.0 × 0.75 mm³

**Altitude Measurement:**
- Standard atmosphere model
- Local pressure reference needed (sea-level calibration)
- Temperature compensation critical
- GPS fusion for absolute altitude

**STMicroelectronics LPS22DF:**
- "Nano" pressure sensor
- Range: 260-1260 hPa
- Accuracy: ±0.5 hPa
- Power: 1.7 μA @ 1 Hz, 200 nA in power-down
- I2C/SPI interface
- FIFO buffer (128 samples)
- Applications: Smartwatches, fitness trackers

**TE Connectivity MS5611:**
- High-resolution barometer
- 24-bit ADC
- Altitude resolution: 10 cm
- Low power: 1 μA standby
- SPI/I2C
- Widely used in drones, weather stations

**Applications:**

**Smartphones:**
- Floor detection in buildings
- Assists GPS (vertical positioning)
- Fitness tracking (stairs climbed)

**Wearables:**
- Elevation gain during hiking
- Swimming depth (water-resistant versions)
- Calorie estimation

**Drones:**
- Altitude hold mode
- Takeoff/landing detection
- Complements barometric with ultrasonic/LiDAR

**Weather Stations:**
- Barometric trend (rising/falling pressure)
- Storm prediction
- Synoptic weather maps

### Air Quality Sensors

#### Gas Sensor Technologies

**1. Metal Oxide Semiconductor (MOS) Sensors:**

**Principle:**
- SnO₂, WO₃ thin film heated to 200-400°C
- Adsorption of reducing gases decreases resistance
- Oxidizing gases increase resistance

**Typical Gases:**
- Reducing: CO, H₂, CH₄, alcohol, NH₃
- Oxidizing: NO₂, O₃

**Advantages:**
- Low cost (<$5)
- Sensitive (ppm to ppb)
- Long lifetime (5+ years)

**Disadvantages:**
- High power (heater: 15-90 mW)
- Slow response (30-60 seconds)
- Cross-sensitivity (poor selectivity)
- Humidity and temperature dependence
- Requires calibration and baseline

**Bosch BME688:**
- AI-capable gas sensor
- SnO₂ sensing element
- Integrated heater (200-400°C)
- AI trained for specific gas profiles
- Output: IAQ (Indoor Air Quality Index) 0-500
- Detects: VOC, CO₂eq, breath VOC
- BSEC library for data fusion
- Power: 0.1-12 mA (heating cycle dependent)

**Sensirion SGP40:**
- MOx-based VOC sensor
- Digital I2C interface
- VOC Index output (0-500)
- Self-calibration algorithm
- Power: 2.6 mA average
- Size: 2.44 × 2.44 × 0.85 mm³

**2. Electrochemical Sensors:**

**Principle:**
- Gas diffuses through membrane to electrode
- Oxidation or reduction reaction
- Current proportional to gas concentration

**Target Gases:**
- CO (carbon monoxide)
- NO₂ (nitrogen dioxide)
- SO₂ (sulfur dioxide)
- O₃ (ozone)
- H₂S (hydrogen sulfide)

**Advantages:**
- High selectivity (specific to target gas)
- Linear response
- Low power (no heater)
- ppm and sub-ppm detection

**Disadvantages:**
- Limited lifetime (2-3 years, electrolyte depletes)
- Expensive ($20-100 per sensor)
- Cross-sensitivity to some gases
- Temperature compensation needed

**Alphasense CO-B4:**
- Carbon monoxide sensor
- Range: 0-1000 ppm
- Sensitivity: 55-90 nA/ppm
- Resolution: <0.5 ppm
- Response time: <30 seconds (t₉₀)
- Lifespan: >5 years
- Applications: Indoor air quality, industrial safety

**3. NDIR (Non-Dispersive Infrared) Sensors:**

**Principle:**
- CO₂ absorbs IR at 4.26 μm wavelength
- IR source → gas chamber → IR detector
- Absorption proportional to CO₂ concentration

**Advantages:**
- Highly selective for CO₂
- Long lifetime (15+ years)
- Stable, no drift
- Not affected by most other gases

**Disadvantages:**
- Expensive ($30-80)
- Higher power (30-300 mW)
- Larger size
- Requires optical path (10-100 mm)

**Sensirion SCD40/SCD41:**
- NDIR CO₂ sensor
- Range: 400-5000 ppm
- Accuracy: ±(50 ppm + 5% reading)
- Resolution: 1 ppm
- Size: 10.1 × 10.1 × 6.5 mm (miniaturized!)
- Power: 15 mA @ 1 measurement/5s
- Automatic self-calibration (ASC)
- I2C interface

**Infineon XENSIV PAS CO2:**
- Photoacoustic spectroscopy (PAS) CO₂ sensor
- Even smaller than NDIR
- Range: 0-10,000 ppm
- Accuracy: ±(30 ppm + 3%)
- Size: 14 × 13.8 × 7.5 mm
- Lower power than NDIR

**4. Particulate Matter (PM) Sensors:**

**Laser Scattering Sensors:**

**Principle:**
- Air drawn through optical chamber by fan
- Laser illuminates particles
- Scattered light detected at 90° angle
- Particle size and count estimated

**Measured Parameters:**
- PM1.0: Particles <1 μm diameter
- PM2.5: Particles <2.5 μm (health concern)
- PM10: Particles <10 μm

**Plantower PMS5003:**
- PM sensor module
- Laser + photodiode detector
- Fan-driven airflow
- Range: 0-500 μg/m³
- Resolution: 1 μg/m³
- UART output
- Power: 100 mA active (fan + laser)
- Size: 50 × 38 × 21 mm
- Cost: ~$10-15

**Sensirion SPS30:**
- Particulate matter sensor
- Optical particle counting
- PM1.0, PM2.5, PM4, PM10 output
- Number concentration and mass concentration
- I2C/UART interface
- Lifetime: >8 years (continuous operation)
- Self-cleaning (fan reversal)
- Accuracy: ±10 μg/m³ @ 0-100 μg/m³

**WHO Air Quality Guidelines (PM2.5):**
- Good: 0-12 μg/m³
- Moderate: 12-35 μg/m³
- Unhealthy for sensitive: 35-55 μg/m³
- Unhealthy: 55-150 μg/m³
- Very unhealthy: 150-250 μg/m³
- Hazardous: >250 μg/m³

#### Indoor Air Quality (IAQ) Monitoring

**Typical IAQ Parameters:**

1. **CO₂ (Carbon Dioxide):**
   - Outdoor: ~420 ppm (2025)
   - Indoor target: <1000 ppm
   - Poor ventilation: >2000 ppm
   - Health effects: Drowsiness, cognitive impairment >1500 ppm

2. **VOC (Volatile Organic Compounds):**
   - Sources: Cleaning products, paints, furniture off-gassing
   - Total VOC (TVOC) measured
   - Good: <220 μg/m³
   - Poor: >660 μg/m³

3. **PM2.5 (Fine Particulate Matter):**
   - Sources: Cooking, smoking, outdoor pollution
   - Target: <12 μg/m³ (24-hour average)
   - High: >35 μg/m³

4. **Humidity:**
   - Optimal: 40-60% RH
   - Low (<30%): Dry skin, static, respiratory issues
   - High (>60%): Mold growth, dust mites

5. **Temperature:**
   - Comfort: 20-24°C
   - Energy efficiency considerations

**IAQ Index Calculation:**
```
Bosch BSEC IAQ Index:
0-50: Excellent
51-100: Good
101-150: Lightly polluted
151-200: Moderately polluted
201-250: Heavily polluted
251-350: Severely polluted
351-500: Extremely polluted

Weighted combination of:
- eCO₂ (equivalent CO₂ from VOC)
- VOC concentration
- Humidity (deviation from optimal)
```

### Ambient Light Sensors (ALS)

#### Light Measurement Units

**Illuminance (lux):**
- Luminous flux per unit area
- Human eye photopic response
- Examples:
  - Direct sunlight: 100,000 lux
  - Overcast day: 10,000 lux
  - Office lighting: 300-500 lux
  - Moonlight: 0.1-1 lux
  - Starlight: 0.001 lux

**Luminous Intensity (candela):**
- Luminous flux per solid angle
- SI base unit for light

**Photodiode vs. ALS:**
- Photodiode: Flat spectral response (IR-sensitive)
- ALS: Filtered to match human eye (CIE photopic curve)

#### Ambient Light Sensor Applications

**Display Brightness Control:**
- Smartphones, tablets, laptops
- Battery saving in low light
- Prevent overly bright screen at night

**ams TSL2591:**
- High-sensitivity ALS
- Range: 188 μlux to 88,000 lux
- 16-bit resolution
- IR compensation (dual photodiodes)
- Programmable gain and integration time
- I2C interface
- Interrupt output (threshold crossing)

**STMicroelectronics VD6281:**
- Ambient light + flicker sensor
- 6-channel color (RGB + IR + clear + UV)
- Flicker detection (100-60 kHz)
- Detect: PWM LED displays, fluorescent lights
- I2C interface

**Automatic Display Adaptation:**
- Measure ambient light
- Adjust backlight brightness
- Adjust contrast/gamma for readability
- Color temperature adjustment (blue light reduction at night)

**Proximity Sensing:**
- Often combined with ALS
- IR LED + photodetector
- Detect nearby objects (0-20 cm)
- Smartphone screen-off during calls

**Vishay VCNL4040:**
- Proximity + ALS sensor
- IR LED @ 940 nm
- Proximity range: 20 cm
- ALS range: 0.0036 to 6553 lux
- I2C interface
- Smart persistence (noise rejection)

### UV (Ultraviolet) Sensors

**UV Index Measurement:**
```
UV Index = Weighted UV irradiance / 25 mW/m²

0-2: Low (minimal protection needed)
3-5: Moderate (seek shade during midday)
6-7: High (protection essential)
8-10: Very high
11+: Extreme (avoid sun exposure)
```

**ams AS7331:**
- UVA + UVB + UVC sensor
- 3-channel spectral measurement
- UVA: 320-400 nm
- UVB: 280-320 nm (sunburn, skin cancer risk)
- UVC: 200-280 nm (blocked by atmosphere normally)
- Applications: Sun exposure monitoring, UV sterilization verification

**Silicon Labs Si1132:**
- UV index sensor
- Ambient light sensor
- Proximity sensor (3-in-1)
- I2C interface
- Wearable-optimized low power

### Multi-Sensor Environmental Modules

**Sensirion SEN55:**
- All-in-one environmental sensor
- PM1.0, PM2.5, PM4, PM10 (optical)
- VOC index
- NOx index (nitrogen oxides)
- Humidity
- Temperature
- I2C interface
- Self-cleaning fan
- Dimensions: 39 × 39 × 12 mm

**Bosch BME688 + BMP390 Combo:**
- Gas (VOC, IAQ)
- Humidity
- Temperature (×2 sensors for validation)
- Pressure (altitude)
- AI-trained for specific applications

**Applications:**

**Smart Home:**
- Trigger ventilation when CO₂ or VOC high
- Adjust HVAC based on temperature/humidity
- Air purifier control based on PM2.5

**Wearables:**
- Environmental awareness
- Allergy alerts (pollen, air quality)
- UV exposure tracking

**Smart Cities:**
- Distributed air quality monitoring
- Heat island mapping
- Pollution source identification

**Agriculture:**
- Greenhouse climate control
- Soil temperature and moisture
- Pest/disease prediction models

### Calibration and Accuracy

#### Field Calibration Techniques

**Temperature Sensors:**
- Ice bath (0°C)
- Boiling water (100°C at sea level, adjust for altitude)
- Calibrated reference thermometer
- Multi-point calibration (3-5 points)

**Humidity Sensors:**
- Salt solution calibration
  - 33% RH: Saturated MgCl₂
  - 75% RH: Saturated NaCl
  - 97% RH: Saturated K₂SO₄
- Sealed container, 24-hour stabilization
- Commercial calibration kits available

**Pressure Sensors:**
- Absolute pressure: Vacuum chamber, known altitude
- Differential: Pressure controller with manometer
- Barometric: GPS-assisted altitude calibration

**Gas Sensors:**
- Zero calibration: Clean air (outdoor, away from sources)
- Span calibration: Known concentration gas cylinder
- MOx sensors: Baseline drift compensation
- CO₂ NDIR: Automatic self-calibration (ASC) assumes 400 ppm minimum

#### Long-Term Stability

**Drift Rates:**

| Sensor Type | Typical Drift | Recalibration Interval |
|-------------|---------------|------------------------|
| Pt100 RTD | <0.01°C/year | 2-5 years |
| Thermistor | <0.05°C/year | 1-2 years |
| Capacitive RH | <0.5% RH/year | 1 year |
| MEMS Pressure | <0.1 hPa/year | Not typically needed |
| MOx Gas | Baseline drift | Weekly auto-calibration |
| Electrochemical | 5-10%/year | 1-2 years or replacement |
| NDIR CO₂ | <2% over lifetime | 5+ years (self-calibrating) |

**Environmental Factors:**
- Temperature cycling: Accelerates aging
- High humidity: Degrades some sensors
- Contaminants: Silicones poison some gas sensors
- UV exposure: Degrades plastics, sensors

---

**References:**
- Sensirion Application Notes
- Bosch Sensortec Environmental Sensor Guides
- EPA Air Quality Monitoring Guidelines
- ASHRAE Standards for IAQ
- ISO/IEC Guide 98-3: Uncertainty of Measurement

© 2025 SmileStory Inc. / WIA
弘익人間 · Benefit All Humanity
