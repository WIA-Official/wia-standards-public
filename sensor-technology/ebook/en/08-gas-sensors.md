# Chapter 8: Gas Sensors and Air Quality Monitoring

## Gas Sensor Technologies Deep Dive

### Metal Oxide Semiconductor (MOX) Sensors

#### Operating Principles

**Semiconductor Gas Sensing:**

**Tin Oxide (SnO₂) - n-type Semiconductor:**
- Operating temperature: 200-400°C (requires heater)
- Surface reactions with reducing gases decrease resistance
- Surface reactions with oxidizing gases increase resistance

**Chemical Reactions:**

**In Air (Baseline):**
```
O₂(gas) → O₂(adsorbed)
O₂(adsorbed) + e⁻ → O⁻(surface)  [T < 150°C]
O₂(adsorbed) + e⁻ → 2O⁻(surface)  [T > 150°C]

Effect: Depletion of electrons from surface, high resistance
```

**Reducing Gas Exposure (e.g., CO):**
```
CO(gas) + O⁻(surface) → CO₂(gas) + e⁻

Effect: Electrons released back to material, resistance decreases
```

**Resistance-Gas Relationship:**
```
R_gas / R_air = a × [Gas]^(-b)

Where:
a, b = Material and temperature-dependent constants
[Gas] = Gas concentration (ppm)

Typical: b = 0.5 to 0.8 for CO, H₂, CH₄
```

**Sensitivity:**
```
S = R_air / R_gas  (for reducing gases)
S = R_gas / R_air  (for oxidizing gases)

Example: Bosch BME688
- CO at 500 ppm: S ≈ 5-10
- Ethanol at 500 ppm: S ≈ 20-50
```

#### Commercial MOX Sensors

**Bosch BME688:**
```
Features:
- Integrated heater (programmable temperature profiles)
- Temperature range: 200-400°C
- Power: 0.1-12 mA (heating cycle dependent)
- Gas scanning: Multiple temperatures in sequence
- AI-based pattern recognition (BSEC library)

Output:
- Raw resistance value
- IAQ index (0-500)
- eCO₂ (equivalent CO₂, 400-60000 ppm)
- bVOC (breath VOC in ppm)

Applications:
- Indoor air quality monitors
- Breath analysis (ketone detection)
- Food quality assessment
```

**Figaro TGS Series:**

**TGS822 (Organic Solvents):**
- Target: Ethanol, toluene, acetone
- Range: 50-5000 ppm
- Heater: 5V, 130 mW
- Analog resistance output
- Low cost (~$5)

**TGS2600 (Air Quality):**
- General air contaminants, hydrogen
- Sensitivity: H₂, CO, low levels
- Fast response: <10 seconds
- Long life: >5 years

**TGS2620 (Alcohol, Solvents):**
- High sensitivity to alcohol vapor
- Breathalyzer applications
- Range: 50-5000 ppm ethanol

**Sensirion SGP40:**
```
MOx-based VOC sensor:
- Digital I2C output
- VOC Index algorithm (0-500)
- Self-calibration over 24 hours
- Low power: 2.6 mA average
- Size: 2.44 × 2.44 × 0.85 mm³
- Temperature/humidity compensation
```

#### Selectivity Challenges

**Cross-Sensitivity:**
MOX sensors respond to multiple gases, making selective detection difficult.

| Sensor | Primary Target | Cross-Sensitivities |
|--------|----------------|---------------------|
| SnO₂ | CO, CH₄, H₂ | Ethanol, acetone, NH₃, H₂S |
| WO₃ | NO₂, O₃ | CO, H₂, reducing gases |
| In₂O₃ | O₃, NO₂ | Humidity sensitive |

**Selectivity Improvement Techniques:**

**1. Temperature Modulation:**
- Cycle heater temperature (200-400°C)
- Gas-specific temperature response fingerprint
- Pattern recognition algorithm
- Example: Bosch BME688 scanning mode

**2. Filters:**
- Activated carbon filter: Remove specific gases
- Zeolite filter: Size-selective (molecular sieving)
- Trade-off: Reduced sensitivity, slower response

**3. Multi-Sensor Arrays:**
- 4-16 sensors with different materials
- Each sensor responds differently to gas mixture
- Machine learning classification
- "Electronic nose" systems

**4. Doping and Catalysts:**
- Noble metals (Pt, Pd, Au) on SnO₂ surface
- Improve selectivity and lower operating temperature
- Pt-doped: Enhanced H₂ sensitivity
- Pd-doped: CH₄ detection

### Electrochemical Gas Sensors

#### Operating Principles

**Three-Electrode Cell:**
```
Components:
1. Working Electrode (WE): Target gas reacts
2. Counter Electrode (CE): Completes circuit
3. Reference Electrode (RE): Stable potential reference

Electrolyte:
- Aqueous acidic or alkaline solution
- Ionic conductor, electronic insulator
```

**Carbon Monoxide Detection:**

**Oxidation Reaction (Working Electrode):**
```
CO + H₂O → CO₂ + 2H⁺ + 2e⁻

Electrons flow through external circuit → current proportional to CO concentration
```

**Reduction Reaction (Counter Electrode):**
```
O₂ + 4H⁺ + 4e⁻ → 2H₂O

Balances the electrochemical cell
```

**Current Output:**
```
I = n × F × (A / t) × (P / RT) × C

Where:
I = Current (A)
n = Number of electrons per molecule (2 for CO)
F = Faraday constant (96485 C/mol)
A = Electrode area
t = Time
P = Pressure
R = Gas constant
T = Temperature
C = Gas concentration

Simplified: I ≈ k × C
Typical sensitivity: 50-90 nA/ppm for CO sensors
```

#### Commercial Electrochemical Sensors

**Alphasense CO-B4 (Carbon Monoxide):**
```
Specifications:
- Range: 0-1000 ppm (also 2000, 5000 ppm versions)
- Sensitivity: 55-90 nA/ppm @ 20°C
- Resolution: <0.5 ppm
- Response time (t₉₀): <30 seconds
- Temperature range: -30°C to +50°C
- Lifetime: >5 years in air
- Zero offset: ±10 ppm equivalent
- Overgas limit: 2000 ppm continuous

Output: Two pins (working and auxiliary electrodes)
- Main signal: WE current
- Auxiliary: Baseline drift compensation
```

**Alphasense NO₂-B43F (Nitrogen Dioxide):**
```
Specifications:
- Range: 0-20 ppm (toxic gas, low concentrations)
- Sensitivity: -175 to -450 nA/ppm
- Resolution: <0.010 ppm
- Response time: <50 seconds
- Cross-sensitivity to NO, O₃ (requires compensation)
```

**Alphasense OX-B431 (Ozone + NO₂):**
```
Oxidizing gas sensor:
- Responds to: O₃, NO₂, Cl₂
- Sensitivity: -275 to -750 nA/ppm (NO₂ equivalent)
- Air quality monitoring
- Water treatment (ozone monitoring)
```

#### Sensor Electronics and Signal Conditioning

**Transimpedance Amplifier:**
```
Op-amp circuit converts sensor current to voltage:

V_out = -I_sensor × R_feedback

Example:
I_sensor = 100 nA (CO at 2 ppm, sensitivity 50 nA/ppm)
R_feedback = 100 kΩ
V_out = -100 nA × 100 kΩ = -10 mV

ADC resolution required:
For 0.1 ppm resolution → 5 nA → 0.5 mV
12-bit ADC (0-4096) → LSB = 1 mV (sufficient)
16-bit ADC (0-65536) → LSB = 0.06 mV (better)
```

**Temperature Compensation:**
```
Sensitivity varies with temperature:
S(T) = S₀ × [1 + α×(T - T₀)]

Typical α ≈ 0.5-1.0% per °C

Example:
S₀ = 60 nA/ppm @ 20°C
At 40°C: S = 60 × [1 + 0.007×(40-20)] = 68.4 nA/ppm (+14%)

Compensation:
1. Measure sensor temperature (NTC thermistor)
2. Apply correction factor in firmware
3. Calibration table or polynomial fit
```

**Baseline Drift Compensation:**
```
Electrochemical sensors exhibit zero drift over time:
- Electrolyte evaporation
- Electrode degradation
- Typical drift: ±5-10 ppm equivalent over 1 year

Compensation strategies:
1. Dual electrode (working + auxiliary)
2. Periodic auto-zero in clean air
3. Manufacturer calibration certificates (update coefficients)
```

### NDIR (Non-Dispersive Infrared) CO₂ Sensors

#### Operating Principles

**IR Absorption:**

**Beer-Lambert Law:**
```
I = I₀ × exp(-α × C × L)

Where:
I = Transmitted intensity
I₀ = Incident intensity
α = Absorption coefficient (specific to gas and wavelength)
C = Gas concentration
L = Optical path length

Rearranged:
C = -ln(I / I₀) / (α × L)
```

**CO₂ Absorption:**
- Strong absorption band: 4.26 μm wavelength
- Other gases (H₂O, CH₄) absorb at different wavelengths
- Narrow-band optical filter isolates 4.26 μm

**Sensor Architecture:**
```
[IR Source] → [Gas Chamber] → [Optical Filter] → [IR Detector]
                    ↓
            (CO₂ absorbs IR)

Dual-channel design:
- Active channel: 4.26 μm (CO₂ absorption)
- Reference channel: 3.95 μm (non-absorbing, compensates drift)
```

#### Commercial NDIR Sensors

**Sensirion SCD40/SCD41:**
```
Miniaturized NDIR CO₂ sensor:
- Measurement range: 400-5000 ppm (SCD40), 400-40000 ppm (SCD41)
- Accuracy: ±(50 ppm + 5% of reading)
- Repeatability: ±10 ppm
- Response time (τ₆₃): 60 seconds
- Operating temperature: 0-50°C
- Humidity: 0-95% RH non-condensing

Size: 10.1 × 10.1 × 6.5 mm³ (miniaturized!)
Power: 15 mA peak, <0.5 mA average @ 5 min sampling

Interface: I2C
- CO₂ concentration (ppm)
- Temperature (°C)
- Humidity (% RH)

Features:
- Automatic Self-Calibration (ASC): Assumes 400 ppm minimum over 7 days
- Field calibration: Forced recalibration (FRC) to known concentration
- Low-power mode: Single-shot measurement, long-term logging
```

**Infineon XENSIV PAS CO2:**
```
Photoacoustic Spectroscopy (PAS):
- IR LED modulated at acoustic frequency
- CO₂ absorbs IR → heats up → pressure wave
- MEMS microphone detects acoustic signal
- Amplitude proportional to CO₂ concentration

Advantages over traditional NDIR:
- Smaller size (14 × 13.8 × 7.5 mm)
- Lower power
- No optical path length constraints

Specifications:
- Range: 0-10,000 ppm (±400 ppm or ±5%)
- Accuracy: ±(30 ppm + 3% of reading)
- Response time: <60 seconds
- I2C interface
```

**Vaisala CARBOCAP®:**
```
High-end NDIR (industrial/scientific):
- Range: 0-20% CO₂ (0-200,000 ppm)
- Accuracy: ±(20 ppm + 1% of reading)
- Pressure and temperature compensation
- Automatic background correction
- Analog 0-10V or 4-20 mA output, digital RS-485

Applications:
- Greenhouses (CO₂ enrichment for plant growth)
- Fermentation monitoring
- Incubators
- Building automation
```

#### Calibration and Maintenance

**Factory Calibration:**
- Two-point calibration (typically 0 ppm and 2000 ppm)
- NIST-traceable gas standards
- Temperature/humidity characterization
- Stored coefficients in EEPROM

**Automatic Self-Calibration (ASC):**
```
Assumption: Sensor exposed to outdoor air (400 ppm CO₂) at least once every 7 days

Algorithm:
1. Track minimum CO₂ reading over 7 days
2. Assume minimum = 400 ppm (current atmospheric CO₂)
3. Adjust calibration offset

Limitations:
- Fails if sensor always indoors (>400 ppm)
- Fails in sealed environment (submarines, spacecraft)
- Manual override required for such applications
```

**Field Calibration (FRC - Forced Recalibration):**
```
Procedure:
1. Expose sensor to known CO₂ concentration (e.g., outdoor air ≈400-420 ppm, or calibration gas)
2. Wait for stable reading (5+ minutes)
3. Send I2C command: Set current reading = reference concentration
4. Sensor updates calibration offsets

Frequency: Every 6-12 months for critical applications
```

### Particulate Matter (PM) Sensors

#### Laser Scattering Sensors

**Operating Principle:**
```
1. Fan draws air through optical chamber
2. Laser diode illuminates particle stream
3. Particles scatter light (Mie scattering)
4. Photodiode at 90° angle detects scattered light
5. Pulse count and height → particle size distribution
6. Algorithm converts to mass concentration (μg/m³)
```

**Scattering Intensity:**
```
I_scattered ∝ d⁶ / λ⁴  (for particles much smaller than wavelength)
I_scattered ∝ d²        (for particles much larger than wavelength)

Where:
d = Particle diameter
λ = Laser wavelength (typically 650 nm red laser)

Result: Larger particles scatter much more light
```

**Particle Size Bins:**
```
Typical output:
- PM1.0: Particles with diameter <1.0 μm
- PM2.5: Particles with diameter <2.5 μm
- PM10: Particles with diameter <10 μm

Also particle counts in size bins:
- 0.3-0.5 μm
- 0.5-1.0 μm
- 1.0-2.5 μm
- 2.5-10 μm
```

#### Commercial PM Sensors

**Plantower PMS5003:**
```
Low-cost laser PM sensor:
- PM1.0, PM2.5, PM10 (μg/m³)
- Particle counts in 6 size bins
- Range: 0-500 μg/m³ (effective), 0-1000 μg/m³ (max)
- Resolution: 1 μg/m³
- Response time: <10 seconds
- Consistency: ±10% (within same batch)
- Absolute accuracy: ±10 μg/m³ and ±10%
- UART output (9600 baud)
- Power: 100 mA active (fan + laser)
- Size: 50 × 38 × 21 mm
- Cost: ~$10-15 (bulk)

Limitations:
- Sensitive to particle composition (calibrated for urban dust)
- Humidity >70% RH causes measurement drift
- Fan noise (40-50 dBA)
```

**Sensirion SPS30:**
```
High-quality optical PM sensor:
- PM1.0, PM2.5, PM4, PM10 mass concentration
- Number concentration (#/cm³) in 5 size bins (0.5, 1.0, 2.5, 4, 10 μm)
- Range: 0-1000 μg/m³
- Accuracy (PM2.5): ±10 μg/m³ @ 0-100 μg/m³, ±10% @ 100-1000 μg/m³
- I2C or UART interface
- Long lifetime: >10 years (>8 years continuous operation)
- Self-cleaning: Automatic fan reversal to blow out accumulated dust
- Industrial-grade reliability

Size: 41 × 41 × 12 mm
Power: 80 mA (measurement mode), 5 mA (idle)
Cost: ~$30-40
```

**Honeywell HPMA115S0:**
```
Compact PM sensor:
- PM2.5, PM10 output
- UART interface
- Auto-clean (heater dries sensor periodically)
- Low power sleep mode
- Lifespan: 3 years
```

#### Health and Regulatory Standards

**PM2.5 Air Quality Index (AQI):**

| PM2.5 (μg/m³) | AQI | Category | Health Implications |
|---------------|-----|----------|---------------------|
| 0-12 | 0-50 | Good | Air quality is satisfactory |
| 12.1-35.4 | 51-100 | Moderate | Acceptable; unusually sensitive people may experience respiratory symptoms |
| 35.5-55.4 | 101-150 | Unhealthy for Sensitive Groups | Sensitive groups (children, elderly, asthma) may experience health effects |
| 55.5-150.4 | 151-200 | Unhealthy | Everyone may begin to experience health effects |
| 150.5-250.4 | 201-300 | Very Unhealthy | Health alert: everyone may experience more serious health effects |
| 250.5+ | 301-500 | Hazardous | Health warnings of emergency conditions |

**WHO Air Quality Guidelines (2021):**
- PM2.5: 15 μg/m³ (24-hour mean), 5 μg/m³ (annual mean)
- PM10: 45 μg/m³ (24-hour mean), 15 μg/m³ (annual mean)

**Sources of PM2.5:**
- Vehicle exhaust (diesel particularly)
- Wood burning (fireplaces, wildfires)
- Coal power plants
- Industrial processes
- Cooking (especially frying, grilling)
- Cigarette smoke
- Dust storms

### Multi-Gas Sensor Arrays

#### Electronic Nose Systems

**Concept:**
- Mimic biological olfactory system
- Array of partially selective sensors
- Pattern recognition algorithm
- Applications: Food quality, disease diagnosis, security

**Typical E-Nose Architecture:**
```
Sensor Array:
- 4-32 gas sensors (MOX, electrochemical, PID)
- Each sensor responds to gas mixture differently
- Create unique "odor fingerprint"

Signal Processing:
- Preprocessing: Baseline correction, normalization
- Feature extraction: Peak amplitude, response time, decay curve
- Dimensionality reduction: PCA (Principal Component Analysis)
- Classification: SVM, neural network, k-NN

Output:
- Gas identification
- Concentration estimation
- Quality score
```

**Commercial Systems:**

**Cyranose 320:**
- 32 polymer composite sensors
- Each sensor: Carbon black + polymer
- Different polymers swell differently with VOCs → resistance change
- Applications: Food spoilage detection, infection diagnosis

**Airsense PEN3:**
- 10 MOX sensors with different dopings
- Metal oxide array: W, Ti, Sn oxides
- Applications: Coffee quality, essential oils, environmental monitoring

#### Applications

**Food Quality:**
```
Fish Freshness:
- Fresh: Low levels of trimethylamine (TMA)
- Spoiled: High TMA, ammonia, sulfur compounds
- E-nose: 95%+ accuracy in freshness classification

Coffee Roast Level:
- Light, medium, dark roast produce different VOC profiles
- Pyrazines, furans, aldehydes
- E-nose can classify roast and detect defects

Fruit Ripeness:
- Ethylene, esters, aldehydes
- Non-destructive testing
- Optimize harvest timing
```

**Medical Diagnostics:**
```
Breath Analysis:
- Diabetes: Acetone (ketone bodies)
- Liver disease: Ammonia, sulfur compounds
- Lung cancer: Specific VOC pattern
- H. pylori infection: Urea breath test (¹³C-labeled)

Challenges:
- Breath variability (food, hydration)
- Small concentration differences
- Regulatory approval (medical device classification)

Status: Research stage, limited clinical deployment
```

**Security and Safety:**
```
Explosives Detection:
- TNT, RDX, PETN vapor
- Trace amounts (ppb-ppt)
- Ion mobility spectrometry (IMS), not simple MOX

Hazardous Material:
- Chemical warfare agents
- Toxic industrial chemicals (TICs)
- Multi-sensor array + AI classification
```

### Future Trends in Gas Sensing

#### Miniaturization and Integration

**MEMS Gas Sensors:**
- Micro-hotplate (100-500 μm)
- Power: <10 mW (vs. 100+ mW for macro heaters)
- Fast heating: <100 ms to operating temperature
- Arrays on single chip
- Example: Cambridge CMOS Sensors CCS811

**Lab-on-a-Chip:**
- Integrate sampling, preconcentration, separation, detection
- Microfluidic channels
- Gas chromatography on a chip (μGC)
- Applications: Portable environmental monitoring, medical diagnostics

#### Novel Sensing Mechanisms

**Graphene Gas Sensors:**
- 2D material, extremely high surface-to-volume ratio
- Conductivity changes with adsorbed molecules
- ppm to ppb sensitivity
- Room temperature operation (no heater)

**Challenges:**
- Manufacturing: Large-area, defect-free graphene
- Reproducibility
- Long-term stability
- Selectivity (surface functionalization needed)

**Status:** Research/early commercialization

**Nanowire Sensors:**
- Si, SnO₂, ZnO nanowires
- High surface area
- Enhanced sensitivity (sub-ppm)
- Arrays for selectivity

**Optical Gas Sensors (Spectroscopy):**
- FTIR (Fourier Transform Infrared): Lab equipment, multi-gas
- Tunable diode laser (TDLS): Specific gases, high sensitivity
- Cavity ring-down spectroscopy (CRDS): ppt sensitivity, expensive

**Miniaturization:** MEMS Fabry-Pérot filters, on-chip spectrometers

#### AI and Machine Learning

**Drift Compensation:**
- MOX sensors drift over months
- ML models learn baseline changes
- Auto-recalibration without reference gas

**Mixture Deconvolution:**
- Multi-gas environment
- Cross-sensitive sensors
- Neural network separates individual gas concentrations
- Example: Distinguish CO vs. ethanol in air quality monitor

**Predictive Maintenance:**
- Sensor health monitoring
- Predict sensor end-of-life
- Optimize replacement schedule

---

**References:**
- Yamazoe, N., & Miura, N. (1992). *Environmental gas sensing*. Sensors and Actuators B: Chemical.
- Alphasense Ltd. Application Notes and Datasheets.
- Sensirion AG Technical Documentation (SCD40, SGP40, SPS30).
- EPA: *Air Quality Index (AQI) Basics*.
- WHO: *Air Quality Guidelines (2021)*.

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
