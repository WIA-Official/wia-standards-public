# PHASE 2 — API

> Smart-textile sensor-and-fiber data path: conductive-fiber
> structures, sensor-embedded textiles, and the health-monitoring
> system API that ingests these readings and projects them into
> clinical-grade signals.

## 5. Conductive Fiber Technologies

### 5.1 Metallic Fibers

#### 5.1.1 Silver-Based Fibers

**Properties:**
- Conductivity: 1×10³ to 1×10⁵ S/m
- Flexibility: Excellent (can be woven and knitted)
- Washability: Good (50-100 cycles with proper encapsulation)
- Biocompatibility: Good (antimicrobial properties)
- Cost: High (4-10 USD/meter for yarn)

**Manufacturing Methods:**
1. **Electroless Plating**: Chemical deposition of silver onto nylon or polyester
2. **Physical Vapor Deposition (PVD)**: Vacuum deposition of silver coating
3. **Sputtering**: High-energy bombardment to deposit silver layer

**Conductivity Calculation:**

```
σ_effective = σ_Ag × f_vol × F_align × F_contact
```

Where:
- σ_Ag = Bulk silver conductivity (6.3×10⁷ S/m)
- f_vol = Volume fraction of silver (0.001-0.1)
- F_align = Alignment factor (0.3-1.0)
- F_contact = Inter-fiber contact factor (0.1-0.5)

**Typical Values:**
- Silver-plated nylon (Ag/PA66): 1×10⁴ S/m
- Silver-plated polyester (Ag/PET): 5×10³ S/m

#### 5.1.2 Copper Fibers

**Properties:**
- Conductivity: 1×10⁶ to 1×10⁷ S/m
- Flexibility: Poor to fair
- Washability: Fair (oxidation concerns)
- Cost: Low (0.5-2 USD/meter)

**Applications:**
- Heating elements in jackets and gloves
- EMI shielding fabrics
- Power transmission in e-textiles

#### 5.1.3 Stainless Steel Fibers

**Properties:**
- Conductivity: 1×10⁶ S/m
- Flexibility: Fair
- Washability: Excellent (>200 cycles)
- Durability: Excellent
- Cost: Low (0.3-1 USD/meter)

**Applications:**
- High-durability applications
- Cut-resistant fabrics
- EMI shielding

### 5.2 Carbon-Based Fibers

#### 5.2.1 Graphene Fibers

**Properties:**
- Conductivity: 1×10⁴ to 1×10⁵ S/m
- Flexibility: Excellent
- Mechanical Strength: Very high (>1 GPa tensile strength)
- Washability: Excellent
- Cost: Very High (>20 USD/meter)

**Production Methods:**
1. **Wet Spinning**: Graphene oxide dispersion → reduction → fiber
2. **Chemical Vapor Deposition (CVD)**: Growth on substrate → transfer
3. **Electrospinning**: Polymer/graphene composite → carbonization

**Applications:**
- High-end sensors requiring flexibility
- Structural health monitoring
- Advanced athletic wear

#### 5.2.2 Carbon Nanotube (CNT) Fibers

**Properties:**
- Conductivity: 1×10⁴ to 1×10⁶ S/m (depending on alignment)
- Flexibility: Excellent
- Strength: Exceptional (>10 GPa theoretical)
- Cost: High (10-30 USD/meter)

**CNT Types:**
- Single-Walled CNT (SWCNT): Higher conductivity, more expensive
- Multi-Walled CNT (MWCNT): Lower cost, good mechanical properties

**Yarn Production:**
1. **Direct Spinning**: From CNT aerogel or forest
2. **Wet Spinning**: CNT dispersion → coagulation
3. **Composite Spinning**: CNT mixed with polymer matrix

### 5.3 Conductive Polymers

#### 5.3.1 PEDOT:PSS

**Properties:**
- Conductivity: 1-10³ S/m (depending on formulation)
- Flexibility: Excellent
- Transparency: Can be optically transparent
- Processability: Solution-based coating/printing
- Stability: Moderate (degradation in water)

**Applications:**
- Flexible electrodes
- Transparent conductors
- Sensor coatings

#### 5.3.2 Polyaniline (PANI)

**Properties:**
- Conductivity: 1-100 S/m
- Environmental Sensitivity: Changes with pH, humidity
- Cost: Low

**Applications:**
- Chemical sensors
- pH-responsive textiles
- Corrosion protection

### 5.4 Conductive Inks and Coatings

#### 5.4.1 Silver Nanoparticle Inks

**Formulation:**
- Silver content: 20-60% by weight
- Particle size: 20-100 nm
- Sintering temperature: 120-200°C
- Sheet resistance: 0.1-1 Ω/sq (after sintering)

**Printing Methods:**
- Screen printing: Line width >50 μm
- Inkjet printing: Line width 20-100 μm
- Spray coating: Large area coverage

#### 5.4.2 Carbon-Based Inks

**Types:**
- Graphene ink: Conductivity 10-100 S/m
- Carbon black ink: Conductivity 1-10 S/m
- CNT ink: Conductivity 100-1000 S/m

**Advantages:**
- Lower cost than silver
- Better flexibility
- No oxidation issues

### 5.5 Fiber Selection Criteria

| Application | Recommended Fiber | Reason |
|-------------|------------------|---------|
| ECG Electrodes | Silver-plated nylon | High conductivity, biocompatibility |
| Strain Sensors | CNT/polymer composite | High gauge factor, flexibility |
| Heating Elements | Stainless steel or copper | Low cost, durability |
| Data Lines | Silver-plated yarn | Signal integrity, flexibility |
| EMI Shielding | Stainless steel | Durability, effectiveness |
| Chemical Sensors | PEDOT:PSS or PANI | Environmental sensitivity |

---


## 6. Sensor-Embedded Textiles

### 6.1 Physiological Sensors

#### 6.1.1 ECG (Electrocardiogram) Sensors

**Principle:**
Detection of electrical signals from cardiac activity through skin-electrode interface.

**Electrode Requirements:**
- Contact area: 5-20 cm²
- Skin-electrode impedance: <10 kΩ at 10 Hz
- Signal frequency: 0.05-150 Hz
- Amplitude: 0.5-4 mV (peak-to-peak)

**Textile Electrode Design:**

```
Electrode Structure:
[Skin] ← [Conductive layer] ← [Adhesive/gel layer] ← [Backing fabric]
```

**Materials:**
- Primary conductor: Silver/silver-chloride coating
- Backing: Polyester or polyamide knit
- Gel layer: Hydrogel or conductive polymer (optional)

**Performance Specifications:**
- Sampling rate: ≥250 Hz (preferably 500-1000 Hz)
- Resolution: 12-24 bits ADC
- Common mode rejection ratio (CMRR): >80 dB
- Input impedance: >10 MΩ

**Lead Configurations:**
- Single-lead: Simple heart rate monitoring
- 3-lead: Basic rhythm analysis
- 12-lead: Comprehensive diagnostic ECG (requires chest strap + limb electrodes)

#### 6.1.2 EMG (Electromyogram) Sensors

**Principle:**
Detection of electrical signals from muscle activation.

**Signal Characteristics:**
- Frequency range: 10-500 Hz
- Amplitude: 50 μV - 5 mV (surface EMG)
- Electrode spacing: 1-2 cm for local muscles

**Applications:**
- Muscle activity monitoring during exercise
- Rehabilitation therapy feedback
- Prosthetic control
- Fatigue detection

**Textile Implementation:**
- Dry electrodes: Silver-plated knit fabric
- Electrode size: 1-2 cm diameter
- Inter-electrode distance: 2 cm (bipolar configuration)

#### 6.1.3 Temperature Sensors

**Sensor Types:**

**A. Thermistor-Embedded Textiles**
- NTC thermistors: Negative temperature coefficient
- Resistance range: 1-100 kΩ at 25°C
- Accuracy: ±0.1-0.5°C
- Response time: 1-10 seconds

**B. Thermocouple-Based Sensors**
- Type K (Chromel-Alumel): -200 to 1350°C
- Type T (Copper-Constantan): -200 to 400°C
- Sensitivity: ~40 μV/°C

**C. Conductive Polymer Sensors**
- Resistance change with temperature
- TCR (Temperature Coefficient of Resistance): 1000-5000 ppm/°C

**Integration Methods:**
1. Embedded thermistors in fabric pockets
2. Printed conductive tracks with temperature-sensitive materials
3. Woven thermocouple junctions

**Calibration:**
```
R(T) = R₀ × exp[β(1/T - 1/T₀)]
```
Where:
- R(T) = Resistance at temperature T (K)
- R₀ = Resistance at reference temperature T₀
- β = Material constant (typically 3000-4000 K)

#### 6.1.4 Respiration Sensors

**Principle:**
Detect chest expansion/contraction during breathing using strain sensors.

**Sensor Design:**

**A. Resistive Strain Sensors**
```
ΔR/R₀ = GF × ε
```
Where:
- GF = Gauge factor (10-100 for textile sensors)
- ε = Strain (0.5-5% during breathing)

**B. Capacitive Sensors**
```
ΔC/C₀ = ε_strain × geometry_factor
```

**C. Piezoelectric Sensors**
- Voltage output proportional to strain rate
- Sensitive to breathing dynamics

**Placement:**
- Chest band: Thoracic breathing
- Abdominal band: Diaphragmatic breathing
- Dual placement: Total respiratory volume

**Signal Processing:**
- Breathing rate: 10-30 breaths/min (normal adult)
- Signal filtering: 0.1-1 Hz bandpass
- Pattern analysis: Apnea detection, irregular breathing

#### 6.1.5 Blood Oxygen (SpO₂) Sensors

**Principle:**
Photoplethysmography (PPG) using red and infrared light to measure oxygen saturation.

**Optical System:**
- Red LED: 660 nm wavelength
- IR LED: 940 nm wavelength
- Photodetector: Silicon photodiode

**Textile Integration Challenges:**
- Stable optical coupling to skin
- Ambient light rejection
- Motion artifact reduction

**Solutions:**
1. Flexible optical windows in fabric
2. Pressure-controlled contact using elastic bands
3. Accelerometer-based motion compensation

**Accuracy:**
- SpO₂ range: 70-100%
- Accuracy: ±2% (90-100% range)
- Heart rate accuracy: ±2 bpm

### 6.2 Physical Sensors

#### 6.2.1 Strain/Stretch Sensors

**Working Principle:**
Resistance or capacitance change with mechanical deformation.

**Sensor Types:**

**A. Resistive Strain Sensors**

Materials:
- CNT/polymer composites: GF = 1-50
- Graphene/polymer composites: GF = 10-100
- Silver nanowire networks: GF = 2-20

**Strain Sensitivity:**
```
GF = (ΔR/R₀) / ε
```

**Hysteresis:**
- Target: <5% over 1000 cycles
- Factors: Viscoelastic recovery, fiber slippage

**B. Capacitive Strain Sensors**

Structure:
```
[Top electrode] ← [Dielectric elastomer] ← [Bottom electrode]
```

Capacitance change:
```
ΔC/C₀ = ε × (2 + ν)
```
Where:
- ε = Applied strain
- ν = Poisson's ratio (~0.5 for elastomers)

**Applications:**
- Joint angle measurement (knees, elbows)
- Posture monitoring
- Gait analysis
- Breathing pattern detection

#### 6.2.2 Pressure Sensors

**Sensor Technologies:**

**A. Resistive Pressure Sensors**
- Piezoresistive materials: Carbon black, CNT, graphene
- Pressure range: 1-500 kPa
- Sensitivity: 0.1-10 kPa⁻¹

**B. Capacitive Pressure Sensors**
- Dielectric foam or elastomer
- Pressure range: 0.1-100 kPa
- Higher linearity than resistive

**C. Piezoelectric Pressure Sensors**
- PVDF (polyvinylidene fluoride) films
- Dynamic pressure measurement
- Self-powered (no bias voltage needed)

**Textile Integration:**
- Sock insoles: Gait analysis, diabetic foot monitoring
- Gloves: Grip force measurement
- Seating: Pressure ulcer prevention
- Mattresses: Sleep position tracking

**Pressure Mapping:**
- Sensor array: 8×8 to 32×32 taxels
- Spatial resolution: 5-10 mm
- Temporal resolution: 10-100 Hz

**Sensitivity Calculation:**
```
S = (ΔR/R₀) / ΔP
```
Where:
- ΔR/R₀ = Relative resistance change
- ΔP = Applied pressure (kPa)
- Typical S: 0.01-1 kPa⁻¹

#### 6.2.3 Humidity/Sweat Sensors

**Principle:**
Electrical property changes (resistance, capacitance) with moisture absorption.

**Sensor Materials:**
- Hygroscopic polymers: Polyimide, polyaniline
- Metal oxides: SnO₂, ZnO
- Carbon-based: Graphene oxide, CNT

**Sweat Rate Measurement:**
```
Sweat_rate (mg/cm²/hr) = ΔRH × calibration_factor
```

**Applications:**
- Hydration monitoring for athletes
- Thermal stress assessment
- Sleep quality analysis
- Wound moisture management

**Performance:**
- RH range: 20-95%
- Response time: <30 seconds
- Accuracy: ±3-5% RH

### 6.3 Chemical and Biosensors

#### 6.3.1 pH Sensors

**Materials:**
- Conductive polymers (PANI, polypyrrole)
- Metal oxide films (IrO₂, RuO₂)

**Response:**
```
E(pH) = E₀ + (RT/nF) × ln([H⁺])
     ≈ E₀ + 59.16 mV × pH (at 25°C)
```

**Applications:**
- Sweat pH monitoring (normal: 4.5-7.0)
- Wound healing assessment
- Skin condition monitoring

#### 6.3.2 Lactate Sensors

**Principle:**
Enzymatic detection (lactate oxidase) with electrochemical transduction.

**Reaction:**
```
Lactate + O₂ →(LOx)→ Pyruvate + H₂O₂
H₂O₂ → 2H⁺ + O₂ + 2e⁻ (at electrode)
```

**Applications:**
- Athletic performance monitoring
- Metabolic disorder detection
- Sepsis early warning

**Performance:**
- Detection range: 0-20 mM (sweat), 0-100 mM (blood through skin)
- Sensitivity: >10 μA/mM
- Response time: <1 minute

#### 6.3.3 Glucose Sensors

**Technologies:**
- Enzymatic (glucose oxidase)
- Non-enzymatic (metal nanoparticles, carbon materials)

**Challenges in Textiles:**
- Enzyme stability during washing
- Interference from sweat composition
- Calibration drift

**Applications:**
- Diabetes management
- Athletic nutrition monitoring

### 6.4 Environmental Sensors

#### 6.4.1 UV Sensors

**Photodetectors:**
- Silicon photodiodes with UV filter
- ZnO, TiO₂ nanostructures
- Organic photodetectors

**UV Index Calculation:**
```
UV_index = (E_erythema × 40 mW/m²) / (25 mW/m²)
```

**Integration:**
- Shoulder patches
- Hat brims
- Wristbands

**Applications:**
- Sun exposure monitoring
- Skin cancer prevention
- Outdoor worker safety

#### 6.4.2 Air Quality Sensors

**Pollutant Detection:**
- CO: Electrochemical sensors
- NO₂: Metal oxide sensors (SnO₂)
- VOCs: MEMS gas sensors
- PM2.5/PM10: Optical particle counters

**Miniaturization Challenges:**
- Power consumption (>50 mW for some sensors)
- Sensor size and weight
- Integration complexity

**Applications:**
- Urban air quality monitoring
- Occupational exposure assessment
- Asthma trigger detection

---


## 9. Health Monitoring Systems

### 9.1 Cardiac Monitoring

#### 9.1.1 Heart Rate Measurement

**Methods:**

**A. ECG-Based**
- R-peak detection in QRS complex
- Accuracy: ±1-2 bpm
- Gold standard for arrhythmia detection

**B. PPG-Based (Photoplethysmography)**
- Optical heart rate sensing
- Accuracy: ±2-5 bpm
- Affected by motion artifacts

**Heart Rate Calculation:**
```
HR (bpm) = 60 / RR_interval (seconds)
```

**Heart Rate Variability (HRV):**
```
SDNN = √(Σ(RR_i - RR_mean)² / (N-1))
```
- Indicator of autonomic nervous system function
- Stress, fitness, recovery assessment

#### 9.1.2 Arrhythmia Detection

**Common Arrhythmias:**
- Atrial Fibrillation (AFib): Irregular RR intervals
- Premature Ventricular Contractions (PVC): Abnormal QRS morphology
- Bradycardia: HR <60 bpm
- Tachycardia: HR >100 bpm

**Detection Algorithms:**
- Template matching
- Machine learning classifiers (SVM, neural networks)
- Rule-based expert systems

**Accuracy:**
- Sensitivity: >90% for AFib detection
- Specificity: >95%

### 9.2 Respiration Monitoring

#### 9.2.1 Measurement Methods

**A. Thoracic Impedance**
- Electrical impedance changes with lung volume
- Z = V / I, ΔZ ∝ ΔLung_volume

**B. Strain-Based**
- Chest/abdominal circumference change
- ΔR/R or ΔC/C

**C. Accelerometer**
- Chest motion during breathing

**Respiration Rate:**
```
RR (breaths/min) = 60 / breathing_cycle_period (seconds)
```

**Normal Values:**
- Adults: 12-20 breaths/min
- Children: 20-30 breaths/min
- Infants: 30-60 breaths/min

#### 9.2.2 Respiratory Patterns

**Pattern Analysis:**
- Regular vs. irregular breathing
- Apnea detection (pause >10 seconds)
- Hyperventilation (RR >25 breaths/min)
- Cheyne-Stokes breathing (crescendo-decrescendo pattern)

**Applications:**
- Sleep apnea screening
- Respiratory disease monitoring (COPD, asthma)
- Stress and anxiety assessment

### 9.3 Temperature Monitoring

#### 9.3.1 Core vs. Skin Temperature

**Core Temperature:**
- Normal: 36.5-37.5°C
- Measurement: Ear, rectal, esophageal (invasive)
- Estimation from skin: Dual heat flux sensors

**Skin Temperature:**
- Normal: 30-34°C (varies by location)
- Direct measurement with textile thermistors

**Heat Flux Method:**
```
T_core = T_skin + (HF / h)
```
Where:
- HF = Heat flux (W/m²)
- h = Heat transfer coefficient

#### 9.3.2 Thermal Stress Assessment

**Heat Stress Indicators:**
- Physiological Strain Index (PSI)
- Wet Bulb Globe Temperature (WBGT)
- Core temperature >38°C: Heat exhaustion risk

**Cold Stress:**
- Skin temperature <30°C: Discomfort
- Core temperature <35°C: Hypothermia

### 9.4 Activity and Posture Monitoring

#### 9.4.1 Accelerometer-Based

**3-Axis Accelerometer:**
- Activity classification: Walking, running, cycling, sitting
- Fall detection: Sudden acceleration >3g
- Sleep tracking: Movement patterns

**Activity Intensity:**
```
Activity_counts = Σ|a(t) - a_mean| (over epoch)
```

**Energy Expenditure:**
```
METs = 1.0 + 0.0175 × Activity_counts
```

#### 9.4.2 Posture Classification

**Strain Sensor Array:**
- Joint angle estimation
- Slouching detection
- Ergonomic assessment

**Applications:**
- Workplace ergonomics
- Rehabilitation therapy
- Sports training

### 9.5 Clinical Applications

#### 9.5.1 Continuous Patient Monitoring

**Hospital Settings:**
- ICU patients: Continuous ECG, respiration, temperature
- Post-surgical monitoring
- Neonatal monitoring

**Home Care:**
- Chronic disease management (heart failure, COPD)
- Elderly care and fall prevention
- Remote patient monitoring (telemedicine)

#### 9.5.2 Disease-Specific Monitoring

**Cardiovascular:**
- Atrial fibrillation screening
- Heart failure decompensation
- Ischemia detection

**Respiratory:**
- COPD exacerbation
- Asthma attack prediction
- Sleep apnea severity

**Metabolic:**
- Diabetes (glucose, activity, stress)
- Obesity management

**Neurological:**
- Epilepsy (seizure detection)
- Parkinson's disease (tremor, gait)

---


