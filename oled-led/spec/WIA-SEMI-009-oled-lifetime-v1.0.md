# WIA-SEMI-009: OLED Lifetime and Reliability
## OLED Lifetime Testing and Burn-in Standard v1.0

**Document Type:** Technology-Specific Standard  
**Related Standard:** WIA-SEMI-009 Core v1.0  
**Release Date:** 2025-01-15  

---

## 1. OLED Lifetime Fundamentals

### 1.1 Lifetime Definitions

**LTxx (Lifetime to xx% luminance):**
- **LT95:** Time to reach 95% of initial luminance
- **LT90:** Time to reach 90% of initial luminance
- **LT50:** Time to reach 50% of initial luminance (half-life)

**Standard Metric:** LT95 is industry standard for OLED display lifetime rating

**Initial Luminance (L₀):**
- Measured after 100-hour stabilization period
- Removes infant mortality effects
- Represents stable, characteristic brightness

### 1.2 Degradation Mechanisms

**Primary Degradation:**
- Chemical degradation of organic materials
- Photo-oxidation (light + oxygen)
- Thermal degradation (heat-accelerated reactions)
- Electrochemical degradation (charge-induced reactions)

**Secondary Effects:**
- Morphological changes (crystallization)
- Interface degradation
- Dopant migration
- Encapsulation failure (moisture ingress)

---

## 2. Lifetime Testing Protocols

### 2.1 Standard Lifetime Test

**Test Conditions:**
- **Temperature:** 25°C ambient (panel stabilizes at 35-50°C depending on brightness)
- **Luminance:** Specified operational luminance (e.g., 200, 500, 1000 Cd/m²)
- **Pattern:** Full white (RGB 255, 255, 255) or application-specific
- **Humidity:** 50% ± 10% RH
- **Duration:** Until LT95 or 10,000+ hours minimum

**Measurement Schedule:**
- Initial: t=0 (after 100h stabilization)
- Early stage: Every 100 hours (first 1,000h)
- Mid stage: Every 250 hours (1,000-5,000h)
- Late stage: Every 500 hours (>5,000h)

**Parameters Measured:**
- Luminance (primary metric)
- Chromaticity (color shift)
- Forward voltage (resistance increase indicator)
- Visual inspection (dark spots, lines)

### 2.2 Accelerated Lifetime Testing

**Purpose:** Predict long-term lifetime in reasonable timeframe

**Acceleration Methods:**

**1. Temperature Acceleration:**
- Test temperature: 60-85°C ambient
- Arrhenius equation: τ₂/τ₁ = exp[(Ea/k) × (1/T₁ - 1/T₂)]
  - τ: Lifetime
  - Ea: Activation energy (eV)
  - k: Boltzmann constant (8.617 × 10⁻⁵ eV/K)
  - T: Absolute temperature (Kelvin)

**Typical Activation Energies:**
- Blue OLED: 0.3-0.6 eV
- Green OLED: 0.4-0.7 eV
- Red OLED: 0.5-0.8 eV

**Example:** 60°C test vs. 25°C use, Ea = 0.5 eV
- AF = exp[(0.5 / 8.617×10⁻⁵) × (1/298 - 1/333)] = exp[5800 × 0.000352] ≈ 8.8×
- 1,000 hours at 60°C ≈ 8,800 hours at 25°C

**2. Luminance Acceleration:**
- Power law: τ₂/τ₁ = (L₁/L₂)ⁿ
  - τ: Lifetime
  - L: Luminance
  - n: Acceleration factor (typically 1.5-2.0, determined empirically)

**Example:** 1000 Cd/m² test vs. 200 Cd/m² use, n = 1.7
- AF = (1000/200)¹·⁷ = 5¹·⁷ ≈ 11.2×
- 1,000 hours at 1000 Cd/m² ≈ 11,200 hours at 200 Cd/m²

**3. Combined Acceleration:**
- AF_total = AF_temperature × AF_luminance
- Example from above: 8.8 × 11.2 ≈ 99×
- 500 hours at 60°C and 1000 Cd/m² ≈ 49,500 hours at 25°C and 200 Cd/m²

### 2.3 Acceleration Model Validation

**Requirements:**
- Test at minimum 3 temperature levels
- Test at minimum 3 luminance levels
- Verify Arrhenius and power law models fit data
- R² > 0.95 for model fits

**Multi-Point Validation:**
- Measure actual lifetime at use conditions (if feasible)
- Compare to predicted lifetime from accelerated tests
- Accuracy requirement: Predicted within ±30% of actual

---

## 3. Lifetime Requirements

### 3.1 Minimum Lifetime by Application

| Application | Min LT95 (Hours) | Test Condition |
|-------------|------------------|----------------|
| Smartphone | 30,000 | 200 Cd/m², 25°C |
| Tablet | 40,000 | 200 Cd/m², 25°C |
| Laptop | 50,000 | 150 Cd/m², 25°C |
| TV (Consumer) | 50,000 | 200 Cd/m², 25°C |
| TV (Premium) | 80,000 | 200 Cd/m², 25°C |
| Monitor (Pro) | 80,000 | 120 Cd/m², 25°C |
| Automotive | 100,000 | 300 Cd/m², 25-60°C cycling |
| Medical | 100,000 | 200 Cd/m², 25°C |
| Signage (Indoor) | 80,000 | 300 Cd/m², 25°C |

**Note:** Tandem OLED targets 2× lifetime of standard OLED for same application

### 3.2 Luminance-Dependent Lifetime

**Lifetime Scaling:**
- Displays must report lifetime at multiple luminance levels
- Or provide power-law exponent (n) for user calculation

**Example Specification:**
- LT95 = 100,000 hours at 100 Cd/m²
- LT95 = 50,000 hours at 200 Cd/m²
- LT95 = 25,000 hours at 400 Cd/m²
- Power law exponent n = 1.7 (derived from above)

---

## 4. Burn-in Testing and Acceptance

### 4.1 Static Pattern Burn-in Test

**Test Pattern Options:**

**Option 1: Logo Pattern**
- Bright logo (e.g., white text/icon) at center
- Logo size: 10-20% of screen area
- Background: 50% gray
- Logo luminance: 400-600 Cd/m²

**Option 2: UI Simulation**
- Simulated user interface with static elements
  - Top status bar
  - Bottom navigation bar
  - Side icons
  - Center changing content area
- Represents realistic smartphone/TV usage

**Option 3: Application-Specific**
- Custom pattern matching intended use
- Examples: News ticker, stock ticker, game HUD

**Test Duration:**
- Standard: 1,000 hours
- Extended: 2,000-5,000 hours
- Automotive/Critical: 10,000+ hours

**Measurement Schedule:**
- Initial (t=0)
- Every 250 hours during test
- Final assessment

### 4.2 Burn-in Uniformity Measurement

**Procedure:**
1. After burn-in test, display uniform gray (50% white)
2. Measure luminance at 25-point grid
3. Calculate uniformity deviation

**Uniformity Calculation:**
- Deviation (%) = (Lmax - Lmin) / Lavg × 100%

**Acceptance Criteria:**

| Application | Max Deviation (1,000h) | Max Deviation (2,000h) |
|-------------|------------------------|------------------------|
| Consumer TV | 8% | 12% |
| Premium TV | 5% | 8% |
| Professional | 3% | 5% |
| Medical/Critical | 2% | 3% |

**Ghost Image Assessment:**
- Visual evaluation: Ghost image of static pattern should not be clearly visible on uniform gray
- Quantitative: Luminance difference at logo vs. background area <5% (consumer), <3% (professional)

---

## 5. Burn-in Mitigation Features

### 5.1 Required Features (Consumer Displays)

**Minimum Required:**
1. **Screen Saver:** Activate after 2-10 minutes of static image
2. **Automatic Power Off:** After 15-60 minutes of inactivity

**Recommended:**
3. **Pixel Shift:** 1-2 pixel movement every 5-10 minutes
4. **Periodic Refresh:** Full-screen pattern periodically (e.g., weekly)

### 5.2 Advanced Features (Premium/Professional)

**Strongly Recommended:**
1. **Logo Detection:** Identify static bright areas, dim selectively
2. **ABL with Spatial Awareness:** Reduce brightness of high APL areas
3. **Compensation Algorithms:** Per-pixel voltage adjustment based on usage history
4. **Real-time Aging Monitoring:** Track pixel usage, predict aging

### 5.3 Compensation Algorithm Requirements

**Functionality:**
- Track luminance/current for each pixel or pixel region
- Adjust drive voltage/current to maintain uniform brightness
- Compensate for both overall aging and differential aging

**Performance:**
- Maintain uniformity <5% after 50,000 hours typical use
- Operate transparently (no user-visible artifacts)

**Data Retention:**
- Aging data stored in non-volatile memory
- Survive power cycles

**Limitations:**
- Algorithm effective until voltage/current reaches maximum
- Disclose effective lifetime with compensation

---

## 6. Blue OLED Special Requirements

### 6.1 Blue Degradation Characterization

**Separate Testing:**
- Blue OLED typically degrades fastest
- Requires dedicated blue lifetime testing

**Blue-Only Pattern:**
- Display full blue (RGB 0, 0, 255)
- Measure blue lifetime separately

**Blue Channel in White:**
- When testing white pattern, track individual RGB sub-pixel aging
- Blue will degrade faster, causing color shift

### 6.2 Blue Lifetime Requirements

**Relative to Green/Red:**
- Blue LT95 typically 1/3 to 1/2 of green/red
- Acceptable if compensation algorithms maintain white point
- Or use larger blue sub-pixels (PenTile) to distribute wear

**Minimum Blue Lifetime:**
- Blue LT95 >20,000 hours at 200 Cd/m² equivalent blue contribution
- For white 200 Cd/m², blue contributes ~70 Cd/m² (varies by technology)

### 6.3 Color Shift Limits

**During Lifetime:**
- Δu'v' < 0.010 over lifetime (consumer)
- Δu'v' < 0.005 over lifetime (professional)

**Compensation:**
- Algorithms should correct color shift
- Report lifetime with and without compensation

---

## 7. Tandem OLED Specific Requirements

### 7.1 Lifetime Enhancement Verification

**Expected Improvement:**
- Tandem OLED should demonstrate 2-3× lifetime vs. single-stack at same brightness
- Or same lifetime at 2× brightness

**Test Validation:**
- Compare tandem vs. single-stack in parallel testing
- Same materials, same conditions
- Verify lifetime ratio matches theoretical prediction

### 7.2 Charge Generation Layer (CGL) Stability

**Specific Test:**
- Monitor voltage vs. time during aging
- CGL degradation causes increased voltage

**Acceptance:**
- Voltage increase <20% over LT95 lifetime
- If voltage rises excessively, CGL may be lifetime limiter

---

## 8. Environmental Stress Testing

### 8.1 Temperature Cycling

**Test:**
- Cycle between temperature extremes
- Consumer: -20°C to +60°C
- Automotive: -40°C to +85°C
- Cycle duration: 4-8 hours per cycle
- Number of cycles: 100-500

**Measurement:**
- Initial and after cycles: Luminance, color, efficiency
- Visual inspection: Delamination, dead pixels

### 8.2 Humidity Testing

**Test:**
- 85°C / 85% RH (85/85 test)
- Duration: 500-1,000 hours
- Non-operating (to test encapsulation)

**Acceptance:**
- No visible dark spots (moisture ingress)
- Luminance degradation <10%
- No electrical failures

### 8.3 Thermal Shock

**Test:**
- Rapid temperature transitions (-40°C to +85°C)
- Transition time: <5 minutes
- Dwell time: 30 minutes each extreme
- Cycles: 100-500

**Acceptance:**
- No cracks, delamination
- Electrical continuity maintained
- Luminance change <5%

---

## 9. Lifetime Reporting Requirements

### 9.1 Datasheet Information

**Mandatory Disclosures:**
1. LT95 lifetime at specified luminance and temperature
2. Acceleration model parameters (Ea, n)
3. Testing duration and extrapolation method
4. Confidence interval on lifetime prediction
5. Compensation algorithm details (if used)

**Example:**
"LT95: 50,000 hours at 200 Cd/m², 25°C ambient. Based on 2,000-hour accelerated testing at 60°C and 500 Cd/m². Acceleration factors: Ea=0.5eV, n=1.7. 95% confidence interval: 40,000-60,000 hours. Compensation algorithm extends effective lifetime to >80,000 hours."

### 9.2 Certification Testing

**For WIA Certification:**
- Submit test data from at least 2,000 hours accelerated aging
- Demonstrate model validation
- Verify compensation algorithm effectiveness
- Burn-in testing (1,000 hours minimum static pattern)

---

## 10. Failure Analysis

### 10.1 Failure Modes

**Catastrophic Failures:**
- Complete non-function
- Cause: Electrical short, driver failure, power supply failure
- Should be extremely rare (<0.01% per 10,000 hours)

**Degradation Failures:**
- Luminance below LT95
- Color shift beyond specification
- Non-uniformity beyond specification

### 10.2 Root Cause Analysis

**When failures occur:**
1. Document failure mode
2. Perform visual inspection
3. Electrical characterization
4. Cross-section analysis (if necessary)
5. Material analysis (FTIR, mass spec, etc.)
6. Determine root cause
7. Implement corrective action

**Feedback Loop:**
- Failure data fed back to materials, design, process teams
- Continuous improvement

---

**Published by:**  
World Certification Industry Association (WIA)  
弘益人間 · Benefit All Humanity

© 2025 SmileStory Inc. / WIA
