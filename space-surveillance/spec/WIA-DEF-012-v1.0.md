# WIA-DEF-012: Space Surveillance Specification v1.0

> **Standard ID:** WIA-DEF-012
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Defense & Space Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Space Situational Awareness](#2-space-situational-awareness)
3. [Tracking Sensors](#3-tracking-sensors)
4. [Orbital Determination](#4-orbital-determination)
5. [Space Catalog Management](#5-space-catalog-management)
6. [Conjunction Assessment](#6-conjunction-assessment)
7. [Collision Risk Analysis](#7-collision-risk-analysis)
8. [Debris Tracking](#8-debris-tracking)
9. [Threat Assessment](#9-threat-assessment)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [Safety Protocols](#11-safety-protocols)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the technical framework and operational procedures for space surveillance systems that monitor satellites, orbital debris, and other space objects to ensure the safety and sustainability of space operations.

### 1.2 Scope

The standard covers:
- Space situational awareness (SSA) concepts and architecture
- Tracking sensor systems and networks
- Orbital determination algorithms and methods
- Space object catalog management
- Conjunction assessment and collision prediction
- Debris tracking and characterization
- Threat detection and attribution

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to promote safe and sustainable space operations for all nations, protecting critical space infrastructure and preserving the space environment for future generations through comprehensive surveillance and collision avoidance.

### 1.4 Terminology

- **SSA (Space Situational Awareness)**: Knowledge of space objects and space environment
- **TLE (Two-Line Element)**: Orbital parameters in standardized format
- **TCA (Time of Closest Approach)**: Time when two objects pass nearest
- **Miss Distance**: Minimum separation between two objects
- **Conjunction**: Close approach between two space objects
- **Pc (Collision Probability)**: Likelihood of collision between objects
- **NORAD ID**: Unique catalog number for tracked objects
- **Orbital Debris**: Non-functional human-made objects in orbit
- **RCS (Radar Cross-Section)**: Measure of object detectability by radar

---

## 2. Space Situational Awareness

### 2.1 SSA Framework

Space Situational Awareness consists of four primary components:

#### 2.1.1 Detection and Tracking
- Identify new objects entering orbit
- Maintain tracking on known objects
- Update orbital parameters continuously
- Classify objects by type and origin

#### 2.1.2 Characterization
- Determine object size and shape
- Estimate mass and composition
- Identify object function and status
- Track orientation and attitude

#### 2.1.3 Conjunction Assessment
- Predict close approaches
- Calculate collision probabilities
- Issue collision warnings
- Recommend avoidance maneuvers

#### 2.1.4 Event Analysis
- Detect fragmentation events
- Identify spacecraft maneuvers
- Monitor anomalous behavior
- Investigate space events

### 2.2 SSA Data Flow

```
Sensor Network → Raw Observations → Track Correlation
                                          ↓
Catalog Update ← Orbital Determination ← Track Processing
      ↓
Conjunction Screening → Risk Assessment → Warning/Alert
```

### 2.3 SSA Performance Metrics

| Metric | Requirement | Rationale |
|--------|-------------|-----------|
| Detection threshold | 10 cm @ LEO | Catastrophic damage potential |
| Tracking accuracy | 10-100 m | Conjunction assessment |
| Update frequency | 24 hours | Orbital decay consideration |
| Conjunction warning | 72 hours | Maneuver planning time |
| False alarm rate | <5% | Operational efficiency |
| Catalog completeness | >95% @ 10 cm | Risk management |

---

## 3. Tracking Sensors

### 3.1 Ground-Based Radar

#### 3.1.1 Phased Array Radar

Technical specifications:
```
Frequency: S-band (2-4 GHz) or UHF (420-450 MHz)
Peak Power: 1-5 MW
Beamwidth: 1-3 degrees
Range: 3,000-5,000 km (LEO), 1,000-2,000 km (MEO)
Accuracy: 10-50 meters (range), 0.1-0.5 deg (angle)
Update Rate: 1-10 Hz
```

Advantages:
- Rapid beam steering (microseconds)
- Multiple simultaneous tracks
- All-weather operation
- High power-aperture product

Limitations:
- Fixed installation
- Limited coverage area
- High cost

#### 3.1.2 Mechanical Radar

Technical specifications:
```
Frequency: S-band (2-4 GHz) or X-band (8-12 GHz)
Peak Power: 0.5-2 MW
Beamwidth: 1-2 degrees
Range: 2,000-4,000 km (LEO)
Accuracy: 20-100 meters
Update Rate: 0.1-1 Hz (scan dependent)
```

Advantages:
- Lower cost than phased array
- Proven technology
- Flexible deployment

Limitations:
- Slower beam positioning
- Mechanical wear and maintenance
- Limited simultaneous tracking

### 3.2 Optical Telescopes

#### 3.2.1 Wide-Field Survey Telescopes

Technical specifications:
```
Aperture: 0.5-1.5 meters
Field of View: 1-10 square degrees
Limiting Magnitude: 17-20 (V-band)
Detection Threshold: 10 cm @ GEO, 1 m @ 40,000 km
Accuracy: 1-10 arcseconds
Cadence: Multiple passes per night
```

Primary uses:
- GEO satellite tracking
- Deep-space object detection
- Debris characterization
- Satellite catalog maintenance

#### 3.2.2 Satellite Laser Ranging (SLR)

Technical specifications:
```
Wavelength: 532 nm (green) or 1064 nm (IR)
Pulse Energy: 10-200 mJ
Pulse Rate: 1-10 kHz
Range Accuracy: 1-10 mm
Maximum Range: 40,000 km
```

Applications:
- Precision orbit determination
- Satellite position verification
- Orbital parameter refinement
- Geophysical research

### 3.3 Space-Based Sensors

#### 3.3.1 Infrared Sensors

Detect objects via:
- Reflected sunlight
- Emitted thermal radiation
- Rocket plume signatures

Advantages:
- Continuous coverage
- No atmospheric interference
- Global field of view

#### 3.3.2 Space Surveillance Satellites

Examples:
- SBSS (Space-Based Space Surveillance) - USA
- Kondor-E - Russia
- SSA satellites - Various nations

Capabilities:
- Track objects in all orbital regimes
- Detect faint objects
- Monitor GEO belt continuously
- Support conjunction assessment

---

## 4. Orbital Determination

### 4.1 Keplerian Elements

Classical orbital elements (COE):

```
a   = Semi-major axis (m)
e   = Eccentricity (dimensionless)
i   = Inclination (degrees)
Ω   = Right Ascension of Ascending Node (RAAN) (degrees)
ω   = Argument of Periapsis (degrees)
M   = Mean Anomaly (degrees)
```

Conversion to state vector:
```
r = [x, y, z]  (position in meters)
v = [vx, vy, vz]  (velocity in m/s)
```

### 4.2 Orbit Propagation

#### 4.2.1 Two-Body Problem

For unperturbed orbits:

```
r̈ + (μ/r³) r = 0
```

Where:
- `μ` = 3.986004418 × 10¹⁴ m³/s² (Earth's gravitational parameter)
- `r` = Position vector
- `r̈` = Acceleration vector

#### 4.2.2 Perturbed Motion

Account for:

1. **Earth's Oblateness (J₂)**:
```
J₂ = 1.08263 × 10⁻³
```

2. **Atmospheric Drag**:
```
a_drag = -½ (ρ v² C_D A/m) v̂
```

Where:
- `ρ` = Atmospheric density (kg/m³)
- `v` = Velocity relative to atmosphere (m/s)
- `C_D` = Drag coefficient (~2.2)
- `A` = Cross-sectional area (m²)
- `m` = Mass (kg)

3. **Solar Radiation Pressure**:
```
a_SRP = (P_☉ C_r A/m) r̂_☉
```

Where:
- `P_☉` = Solar pressure (4.56 × 10⁻⁶ N/m²)
- `C_r` = Reflectivity coefficient (1-2)

4. **Third-body Perturbations**:
- Lunar gravity
- Solar gravity

### 4.3 Orbit Determination Algorithms

#### 4.3.1 Least Squares Batch Processing

Minimize residuals:
```
J = Σᵢ (yᵢ - H(xᵢ))ᵀ Rᵢ⁻¹ (yᵢ - H(xᵢ))
```

Where:
- `yᵢ` = Observation vector
- `H(xᵢ)` = Predicted observation
- `Rᵢ` = Observation covariance matrix

#### 4.3.2 Extended Kalman Filter (EKF)

State prediction:
```
x̂ₖ|ₖ₋₁ = f(x̂ₖ₋₁|ₖ₋₁, uₖ)
Pₖ|ₖ₋₁ = Fₖ Pₖ₋₁|ₖ₋₁ Fₖᵀ + Qₖ
```

State update:
```
Kₖ = Pₖ|ₖ₋₁ Hₖᵀ (Hₖ Pₖ|ₖ₋₁ Hₖᵀ + Rₖ)⁻¹
x̂ₖ|ₖ = x̂ₖ|ₖ₋₁ + Kₖ(yₖ - h(x̂ₖ|ₖ₋₁))
Pₖ|ₖ = (I - Kₖ Hₖ) Pₖ|ₖ₋₁
```

### 4.4 Accuracy Requirements

| Orbital Regime | Position Accuracy | Velocity Accuracy |
|----------------|-------------------|-------------------|
| LEO (400 km) | 10-100 m | 1-10 m/s |
| MEO (20,000 km) | 100-500 m | 0.1-1 m/s |
| GEO (35,786 km) | 500-2,000 m | 0.1-0.5 m/s |

---

## 5. Space Catalog Management

### 5.1 Catalog Structure

Each catalog entry contains:

```json
{
  "objectId": "NORAD-25544",
  "intlDesignator": "1998-067A",
  "name": "ISS (ZARYA)",
  "country": "ISS",
  "launchDate": "1998-11-20",
  "site": "TTMTR",
  "decayDate": null,
  "period": 92.68,
  "inclination": 51.64,
  "apogee": 421,
  "perigee": 418,
  "rcs": "LARGE",
  "objectType": "PAYLOAD",
  "status": "OPERATIONAL",
  "tle": {
    "line1": "1 25544U 98067A   24360.12345678  .00012345  00000-0  12345-3 0  9993",
    "line2": "2 25544  51.6400 123.4567 0001234  89.0123 271.0456 15.54123456123456"
  }
}
```

### 5.2 Object Classification

#### 5.2.1 Object Types
- `PAYLOAD`: Operational satellite or spacecraft
- `ROCKET BODY`: Spent rocket stage
- `DEBRIS`: Fragmentation debris
- `UNKNOWN`: Unidentified object

#### 5.2.2 Status Categories
- `OPERATIONAL`: Active and functional
- `NONOPERATIONAL`: Inactive but intact
- `DECAYED`: Re-entered atmosphere
- `FRAGMENTED`: Broken up in orbit
- `UNKNOWN`: Status undetermined

#### 5.2.3 Size Categories (RCS)
- `LARGE`: >1 m² RCS
- `MEDIUM`: 0.1-1 m² RCS
- `SMALL`: <0.1 m² RCS

### 5.3 Catalog Maintenance

#### 5.3.1 New Object Detection
1. Sensor detects uncorrelated track
2. Track processed to initial orbit
3. Orbit propagated and re-observed
4. Object correlation across sensors
5. Catalog entry created
6. TLE published

#### 5.3.2 Catalog Updates
- Daily TLE updates for active satellites
- Weekly updates for debris
- Event-driven updates for maneuvers
- Decay predictions for low-perigee objects

### 5.4 TLE Format

#### Line 1:
```
1 NNNNNC NNNNNAAA NNNNN.NNNNNNNN +.NNNNNNNN +NNNNN-N +NNNNN-N N NNNNN
```

#### Line 2:
```
2 NNNNN NNN.NNNN NNN.NNNN NNNNNNN NNN.NNNN NNN.NNNN NN.NNNNNNNNNNNNNN
```

---

## 6. Conjunction Assessment

### 6.1 Screening Process

#### 6.1.1 Primary Screening

For each satellite, screen against all catalog objects:

```python
def screen_conjunctions(primary_obj, catalog, time_window):
    conjunctions = []

    for secondary_obj in catalog:
        if primary_obj.id == secondary_obj.id:
            continue

        # Propagate both objects
        r1, v1 = propagate(primary_obj, time_window)
        r2, v2 = propagate(secondary_obj, time_window)

        # Find closest approach
        tca, miss_distance = find_closest_approach(r1, v1, r2, v2)

        # Check threshold
        if miss_distance < SCREENING_THRESHOLD:
            conjunctions.append({
                'primary': primary_obj.id,
                'secondary': secondary_obj.id,
                'tca': tca,
                'miss_distance': miss_distance
            })

    return conjunctions
```

#### 6.1.2 Screening Thresholds

| Orbital Regime | Distance Threshold | Time Window |
|----------------|-------------------|-------------|
| LEO | 5 km | 7 days |
| MEO | 10 km | 7 days |
| GEO | 50 km | 7 days |

### 6.2 Conjunction Analysis

#### 6.2.1 Miss Distance Calculation

In the Radial-Transverse-Normal (RTN) frame:

```
Primary position: r₁ = [r_R1, r_T1, r_N1]
Secondary position: r₂ = [r_R2, r_T2, r_N2]

Relative position: Δr = r₂ - r₁
Miss distance: d = |Δr(t_TCA)|
```

#### 6.2.2 Time of Closest Approach

Minimize distance function:

```
d(t) = |r₂(t) - r₁(t)|

Find t_TCA where dd/dt = 0
```

Analytical solution:
```
t_TCA = t₀ - (Δr · Δv) / (Δv · Δv)
```

Where:
- `Δr` = Relative position vector
- `Δv` = Relative velocity vector

### 6.3 Warning Criteria

Issue conjunction warning if:

```
(miss_distance < threshold) AND
(collision_probability > Pc_threshold) AND
(time_to_TCA < warning_leadtime)
```

Standard thresholds:
- Miss distance: 1 km (LEO), 5 km (GEO)
- Pc threshold: 1 × 10⁻⁴
- Warning leadtime: 72 hours

---

## 7. Collision Risk Analysis

### 7.1 Collision Probability

#### 7.1.1 Foster Method

For circular covariance ellipsoids:

```
Pc = 1 - exp(-R²/2σ²)
```

Where:
- `R` = Combined hard-body radius
- `σ` = Standard deviation of miss distance

#### 7.1.2 Chan Method (2D)

```
Pc = (1/2πσ_xσ_y) ∫∫_A exp(-(x²/2σ_x² + y²/2σ_y²)) dx dy
```

Where:
- `A` = Combined cross-sectional area
- `σ_x, σ_y` = Position uncertainties

#### 7.1.3 Monte Carlo Simulation

For complex covariances:

```python
def calculate_pc_monte_carlo(primary, secondary, n_samples=10000):
    collisions = 0
    combined_radius = (primary.radius + secondary.radius)

    for i in range(n_samples):
        # Sample from position covariance
        r1 = sample_multivariate_normal(primary.position, primary.covariance)
        r2 = sample_multivariate_normal(secondary.position, secondary.covariance)

        # Check collision
        if np.linalg.norm(r2 - r1) < combined_radius:
            collisions += 1

    return collisions / n_samples
```

### 7.2 Risk Assessment Matrix

| Pc Range | Risk Level | Action |
|----------|------------|--------|
| Pc > 1 × 10⁻² | CRITICAL | Immediate maneuver required |
| 1 × 10⁻³ < Pc < 1 × 10⁻² | HIGH | Maneuver strongly recommended |
| 1 × 10⁻⁴ < Pc < 1 × 10⁻³ | MEDIUM | Monitor closely, prepare maneuver |
| 1 × 10⁻⁵ < Pc < 1 × 10⁻⁴ | LOW | Continue monitoring |
| Pc < 1 × 10⁻⁵ | MINIMAL | Routine tracking |

### 7.3 Maneuver Planning

#### 7.3.1 Delta-V Calculation

For radial maneuver:
```
Δv = √(μ/r) × Δh/r
```

For tangential maneuver:
```
Δv = √(μ/r) × (√(r_new/r) - 1)
```

Typical avoidance maneuver: 0.5-5 m/s

#### 7.3.2 Maneuver Optimization

Minimize:
- Fuel consumption (Δv)
- Mission impact
- Collision probability after maneuver

Subject to:
- Pc_after < Pc_threshold
- Miss distance > safety margin
- Operational constraints

---

## 8. Debris Tracking

### 8.1 Debris Environment

#### 8.1.1 Population Estimates

| Size Range | Estimated Number | Detection Capability |
|------------|------------------|---------------------|
| >10 cm | 34,000 | Tracked |
| 1-10 cm | 900,000 | Partially tracked |
| 1 mm - 1 cm | 130 million | Statistical models |
| <1 mm | Trillions | Damage models only |

#### 8.1.2 Spatial Distribution

Peak debris density:
- LEO: 800-1,000 km altitude
- GEO: ±15° of geostationary arc

### 8.2 Debris Sources

Historical fragmentation events:

1. **Collision Events**:
   - Cosmos 2251 / Iridium 33 (2009): 2,000+ fragments
   - Fengyun-1C ASAT test (2007): 3,500+ fragments

2. **Explosion Events**:
   - Upper stage explosions: 250+ events
   - Battery explosions: 50+ events

3. **Degradation**:
   - Paint flakes
   - Multilayer insulation (MLI)
   - Ejecta from impacts

### 8.3 Debris Characterization

#### 8.3.1 Radar Cross-Section (RCS)

Estimate size from RCS:
```
d ≈ √(RCS / π)  (for spherical assumption)
```

Typical RCS values:
- 10 cm sphere: 0.008 m²
- 1 m sphere: 0.8 m²
- Large satellite: 10-100 m²

#### 8.3.2 Optical Magnitude

Estimate size from brightness:
```
d = 10^((H - 15.618 - log₁₀(p_v))/2.5)
```

Where:
- `H` = Absolute magnitude
- `p_v` = Geometric albedo (~0.1-0.2)

### 8.4 Debris Mitigation

International guidelines:
- Post-mission disposal within 25 years (LEO)
- GEO graveyard orbit (+300 km)
- Passivation of energy sources
- Minimize mission-related debris

---

## 9. Threat Assessment

### 9.1 Anomaly Detection

#### 9.1.1 Maneuver Detection

Indicators:
- Unexpected TLE changes
- Orbit altitude change >1 km
- Inclination change >0.1°
- Rapid RAAN drift

Detection algorithm:
```python
def detect_maneuver(tle_history):
    for i in range(1, len(tle_history)):
        delta_a = abs(tle_history[i].sma - tle_history[i-1].sma)
        delta_e = abs(tle_history[i].ecc - tle_history[i-1].ecc)
        delta_i = abs(tle_history[i].inc - tle_history[i-1].inc)

        if delta_a > THRESHOLD_A or delta_i > THRESHOLD_I:
            return {
                'maneuver_detected': True,
                'time': tle_history[i].epoch,
                'delta_v_estimate': estimate_delta_v(delta_a, delta_e, delta_i)
            }

    return {'maneuver_detected': False}
```

#### 9.1.2 Proximity Operations

Suspicious behaviors:
- Approaching within 100 km of high-value satellite
- Matching orbital parameters
- Station-keeping near target
- Multiple rendezvous attempts

### 9.2 Anti-Satellite (ASAT) Detection

#### 9.2.1 Direct-Ascent ASAT

Detection:
- Launch detection via infrared sensors
- Trajectory analysis
- Impact prediction
- Post-impact debris cloud characterization

#### 9.2.2 Co-Orbital ASAT

Indicators:
- Launch into similar orbit as target
- Gradual approach maneuvers
- Rendezvous geometry
- Anomalous payload deployment

### 9.3 Attribution Analysis

Determine:
- Launch site and country
- Orbital characteristics
- Object function and capability
- Ownership and control

Data sources:
- Launch records
- Orbital parameters
- Radio frequency emissions
- Optical signatures
- Diplomatic notifications

---

## 10. Implementation Guidelines

### 10.1 Required Components

Any WIA-DEF-012 compliant system must include:

1. **Sensor Network**: Ground and/or space-based tracking
2. **Data Processing**: Orbit determination and correlation
3. **Catalog Database**: Comprehensive object database
4. **Conjunction Screening**: Automated collision prediction
5. **Risk Assessment**: Collision probability calculation
6. **Warning System**: Timely alerts to operators
7. **Data Distribution**: Sharing with authorized entities

### 10.2 API Interface

#### 10.2.1 Track Space Object
```typescript
interface TrackingRequest {
  objectId: string;
  sensorNetwork: string;
  observationTime: Date;
  position: Vector3;
  velocity: Vector3;
  uncertainty?: CovarianceMatrix;
}

interface TrackingResult {
  objectId: string;
  orbitalElements: KeplerianElements;
  altitude: number;
  period: number;
  inclination: number;
  nextUpdate: Date;
}
```

#### 10.2.2 Calculate Conjunction
```typescript
interface ConjunctionRequest {
  primary: string;
  secondary: string;
  timeWindow: number;  // seconds
  minDistance: number; // meters
}

interface ConjunctionResult {
  id: string;
  primary: string;
  secondary: string;
  timeOfClosestApproach: Date;
  missDistance: number;
  relativeVelocity: number;
  radialDistance: number;
  alongTrackDistance: number;
  crossTrackDistance: number;
}
```

#### 10.2.3 Assess Collision Risk
```typescript
interface RiskAssessmentRequest {
  conjunctionId: string;
  primarySize: number;    // meters
  secondarySize: number;  // meters
  positionUncertainty: number; // meters
}

interface RiskAssessmentResult {
  probability: number;
  riskLevel: 'MINIMAL' | 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  recommendedAction: string;
  maneuverRequired: boolean;
  estimatedDeltaV?: number;
}
```

### 10.3 Data Formats

#### 10.3.1 TLE Format (Standard)
As defined in section 5.4

#### 10.3.2 OMM (Orbit Mean-Elements Message)
XML/JSON format for orbit data exchange

#### 10.3.3 CDM (Conjunction Data Message)
CCSDS standard for conjunction information

### 10.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| S001 | Object not in catalog | Update catalog |
| S002 | Insufficient tracking data | Request more observations |
| S003 | Orbit determination failed | Check input data quality |
| S004 | Conjunction threshold exceeded | Issue warning |
| S005 | Sensor unavailable | Use alternate sensor |
| S006 | Data quality poor | Flag for review |

---

## 11. Safety Protocols

### 11.1 Operational Procedures

#### 11.1.1 Conjunction Warning Process

1. **Detection** (T - 7 days):
   - Automated screening identifies potential conjunction
   - Initial risk assessment performed
   - Entry logged in tracking system

2. **Analysis** (T - 5 days):
   - Refine orbits with additional observations
   - Update collision probability
   - Assess maneuver options

3. **Notification** (T - 72 hours):
   - Issue formal conjunction warning
   - Notify satellite operators
   - Coordinate maneuver if needed

4. **Monitoring** (T - 24 hours):
   - Continuous tracking updates
   - Final Pc calculation
   - Execute maneuver if required

5. **Post-Event** (T + 24 hours):
   - Verify miss distance
   - Update catalog
   - Document lessons learned

### 11.2 Collision Avoidance Criteria

Execute maneuver if:
```
(Pc > 1 × 10⁻⁴) OR
(miss_distance < 1 km AND uncertainty > 500 m) OR
(operator_risk_tolerance exceeded)
```

### 11.3 Data Quality Standards

Required accuracy:
- Position: <100 m (1-sigma)
- Velocity: <1 m/s (1-sigma)
- Observation cadence: Daily minimum
- Covariance realism: Validated against actual errors

### 11.4 Emergency Procedures

#### High-Risk Conjunction (Pc > 1 × 10⁻³)

1. Immediate notification to operators
2. 24/7 monitoring initiated
3. Maneuver planning team activated
4. Backup communication channels verified
5. Post-maneuver tracking arranged

#### Debris-Generating Event

1. Enhanced surveillance of event location
2. Initial debris catalog generation
3. Propagate debris cloud evolution
4. Screen all assets against new debris
5. Issue warnings for high-risk conjunctions
6. Update long-term debris models

---

## 12. References

### 12.1 Standards and Guidelines

1. ISO 24113: Space systems - Space debris mitigation requirements
2. IADC Space Debris Mitigation Guidelines (2020)
3. UN COPUOS Space Debris Mitigation Guidelines (2007)
4. CCSDS Conjunction Data Message (CDM) Standard
5. CCSDS Orbit Data Messages (ODM) Standard

### 12.2 Scientific Papers

1. Kessler, D.J. & Cour-Palais, B.G. (1978). "Collision Frequency of Artificial Satellites"
2. 선행 연구. "Probability of Collision Error Analysis"
3. Chan, F.K. (2008). "Spacecraft Collision Probability"
4. Vallado, D.A. (2013). "Fundamentals of Astrodynamics and Applications"
5. Hoots, F.R. & Roehrich, R.L. (1980). "Spacetrack Report No. 3: SGP4/SDP4"

### 12.3 Physical Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Earth's gravitational parameter | μ | 3.986004418 × 10¹⁴ m³/s² |
| Earth's radius | R⊕ | 6.378137 × 10⁶ m |
| Earth's J₂ | J₂ | 1.08263 × 10⁻³ |
| Speed of light | c | 2.998 × 10⁸ m/s |
| Solar radiation pressure (1 AU) | P☉ | 4.56 × 10⁻⁶ N/m² |

### 12.4 WIA Standards

- WIA-INTENT: Intent-based control systems
- WIA-OMNI-API: Universal API gateway
- WIA-SOCIAL: Inter-agency coordination
- WIA-DEF-010: Military satellite operations
- WIA-AIR-SHIELD: Integrated air defense

---

## Appendix A: Example Calculations

### A.1 Orbital Period Calculation

```
Given:
- Semi-major axis: a = 6,778,000 m (400 km altitude)
- Earth's μ = 3.986004418 × 10¹⁴ m³/s²

Calculation:
T = 2π√(a³/μ)
T = 2π√((6.778×10⁶)³ / 3.986×10¹⁴)
T = 2π√(3.109×10²⁰ / 3.986×10¹⁴)
T = 2π√(7.802×10⁵)
T = 2π × 883.3
T = 5,549 seconds ≈ 92.5 minutes
```

### A.2 Miss Distance from TLE

```
Given:
- Object 1 TLE at epoch
- Object 2 TLE at epoch
- Target time: 2024-12-27 12:00:00 UTC

Steps:
1. Propagate Object 1 to target time using SGP4
2. Propagate Object 2 to target time using SGP4
3. Calculate position difference:
   Δr = r₂ - r₁ = [Δx, Δy, Δz]
4. Compute miss distance:
   d = |Δr| = √(Δx² + Δy² + Δz²)

Example result:
d = 847 meters
```

### A.3 Collision Probability (Simple Case)

```
Given:
- Miss distance: d = 500 m
- Combined hard-body radius: R = 15 m
- Position uncertainty: σ = 100 m

Calculation (Foster method):
Pc = 1 - exp(-R²/2σ²)
Pc = 1 - exp(-(15)²/(2×100²))
Pc = 1 - exp(-225/20000)
Pc = 1 - exp(-0.01125)
Pc = 1 - 0.9888
Pc ≈ 0.0112 = 1.12 × 10⁻²

Risk Level: CRITICAL (maneuver required)
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-DEF-012 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
