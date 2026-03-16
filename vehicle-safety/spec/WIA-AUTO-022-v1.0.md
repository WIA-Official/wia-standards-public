# WIA-AUTO-022: Vehicle Safety Specification v1.0

> **Standard ID:** WIA-AUTO-022
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Safety Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Active Safety Systems](#2-active-safety-systems)
3. [Passive Safety Systems](#3-passive-safety-systems)
4. [Crash Testing Standards](#4-crash-testing-standards)
5. [Occupant Protection](#5-occupant-protection)
6. [Pedestrian Safety](#6-pedestrian-safety)
7. [Child Safety Systems](#7-child-safety-systems)
8. [Data Formats](#8-data-formats)
9. [API Interface](#9-api-interface)
10. [Safety Protocols](#10-safety-protocols)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive vehicle safety standards covering active safety systems, passive protection mechanisms, crash testing protocols, and safety validation frameworks. The standard aims to reduce traffic fatalities and injuries through systematic safety engineering.

### 1.2 Scope

The standard covers:
- Active safety systems (collision avoidance, stability control)
- Passive safety systems (airbags, seatbelts, structures)
- Crash testing methodologies and metrics
- Occupant protection mechanisms
- Pedestrian and vulnerable road user protection
- Child safety systems and restraints
- Safety data formats and APIs
- Compliance and certification protocols

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard is dedicated to saving lives and reducing injuries through advanced vehicle safety technology. Every feature, test, and requirement is designed with the ultimate goal of protecting all road users.

### 1.4 Terminology

- **Active Safety**: Systems that prevent or mitigate crashes before they occur
- **Passive Safety**: Systems that protect occupants during a crash
- **HIC**: Head Injury Criterion - measure of head impact severity
- **Crumple Zone**: Structural area designed to deform and absorb energy
- **NCAP**: New Car Assessment Program - crash test rating system
- **AEB**: Autonomous Emergency Braking
- **ESC**: Electronic Stability Control
- **ISOFIX**: International standard for child seat attachment

---

## 2. Active Safety Systems

### 2.1 Autonomous Emergency Braking (AEB)

#### 2.1.1 System Architecture

**Components:**
1. Forward-facing radar (77 GHz, 150m range)
2. Forward camera (wide-angle, object detection)
3. Processing unit (real-time threat assessment)
4. Brake actuator (hydraulic/electric)

**Detection Algorithm:**
```
TTC = distance / relative_velocity
```

Where:
- `TTC` = Time To Collision (seconds)
- `distance` = Range to obstacle (meters)
- `relative_velocity` = Closing speed (m/s)

**Activation Threshold:**
```
if TTC < 1.5 seconds and driver_input == false:
    issue_warning()
    if TTC < 0.8 seconds:
        apply_emergency_braking()
```

#### 2.1.2 Performance Requirements

| Parameter | Requirement |
|-----------|-------------|
| Detection Range | 150 meters minimum |
| Reaction Time | < 100 milliseconds |
| Braking Force | Up to 1.0 g deceleration |
| Speed Range | 5 - 80 km/h (urban AEB) |
| False Positive Rate | < 0.1% per 1000 km |

#### 2.1.3 Test Scenarios

**Euro NCAP AEB Tests:**
1. Car-to-Car Rear Stationary (CCRs): 10-50 km/h
2. Car-to-Car Rear Moving (CCRm): 30-70 km/h
3. Car-to-Car Rear Braking (CCRb): 50 km/h approach
4. Car-to-Pedestrian (day/night): 20-60 km/h

**Success Criteria:**
- No collision at speeds ≤ 40 km/h
- Speed reduction ≥ 50% at speeds > 40 km/h

### 2.2 Electronic Stability Control (ESC)

#### 2.2.1 System Components

1. **Sensors:**
   - Steering angle sensor (±1080°)
   - Yaw rate sensor (±100°/s)
   - Lateral acceleration sensor (±1.5 g)
   - Wheel speed sensors (4 wheels)

2. **Actuators:**
   - ABS hydraulic unit
   - Individual wheel brake control
   - Engine torque management interface

#### 2.2.2 Control Algorithm

**Yaw Rate Error:**
```
ε_yaw = yaw_rate_desired - yaw_rate_actual

yaw_rate_desired = (steering_angle × velocity) / (wheelbase × (1 + K × velocity²))
```

Where:
- `K` = Understeer coefficient
- `wheelbase` = Distance between axles

**Intervention Logic:**
```
if |ε_yaw| > threshold_1:
    reduce_engine_torque()
    if |ε_yaw| > threshold_2:
        apply_selective_braking()
```

#### 2.2.3 Performance Standards

**FMVSS 126 Requirements:**
- Lateral displacement ≤ 1.22 meters during sine-with-dwell test
- Yaw rate ratio ≤ 1.35 or 0.65
- Response time < 200 milliseconds

### 2.3 Lane Departure Warning (LDW) / Lane Keeping Assist (LKA)

#### 2.3.1 Lane Detection

**Vision System:**
- Forward camera (60° FOV)
- Lane marking detection (Hough transform)
- Tracking range: 3-60 meters

**Lane Model:**
```
y = C₀ + C₁x + C₂x² + C₃x³
```

Where:
- `y` = Lateral position
- `x` = Longitudinal distance
- `C₀-C₃` = Lane polynomial coefficients

#### 2.3.2 Warning/Intervention Criteria

**Time to Lane Crossing (TLC):**
```
TLC = lateral_offset / lateral_velocity
```

**LDW Activation:**
- TLC < 0.5 seconds
- Vehicle speed > 60 km/h
- Turn signal not active
- Driver hands on wheel (torque sensor)

**LKA Intervention:**
- Progressive steering torque: 2-4 Nm
- Gentle correction to lane center
- Override by driver input (> 5 Nm)

### 2.4 Blind Spot Monitoring (BSM)

#### 2.4.1 Detection Zones

**Radar Coverage:**
- Rear quarter panels (24 GHz radar)
- Detection range: 0.5 - 3 meters lateral
- Longitudinal range: -3 to +3 meters from driver

**Alert Levels:**
1. **Passive**: Visual indicator (side mirror LED)
2. **Active**: Visual + auditory warning
3. **Intervention**: Steering correction (if lane change attempted)

#### 2.4.2 Performance Requirements

| Metric | Requirement |
|--------|-------------|
| Detection Probability | > 99% for vehicles |
| False Alarm Rate | < 1 per 100 km |
| Response Time | < 200 ms |
| Speed Range | 30 - 250 km/h |

---

## 3. Passive Safety Systems

### 3.1 Airbag Systems

#### 3.1.1 Frontal Airbags

**Driver Airbag:**
- Volume: 60-70 liters
- Deployment time: 25-35 milliseconds
- Inflation pressure: 100-150 kPa
- Deployment threshold: 20-25 km/h impact

**Passenger Airbag:**
- Volume: 120-150 liters
- Deployment time: 30-40 milliseconds
- Two-stage deployment (occupant sensing)

**Deployment Criteria:**
```
if crash_pulse_integral > threshold and crash_severity > minimum:
    deploy_airbag(stage=calculate_optimal_stage())
```

**Crash Pulse Integral:**
```
∫₀ᵗ a(t) dt > threshold
```

Where `a(t)` is longitudinal acceleration

#### 3.1.2 Side Airbags

**Types:**
1. **Torso Bags**: 12-15 liters, 10-15ms deployment
2. **Curtain Bags**: 20-30 liters, 15-25ms deployment
3. **Pelvis Bags**: 5-8 liters, seat-mounted

**Side Impact Detection:**
- Door pressure sensors
- Lateral accelerometers (B-pillar)
- Deployment threshold: 15-20 km/h lateral impact

#### 3.1.3 Advanced Airbag Features

**Occupant Classification:**
- Weight sensors (seat mat)
- Position sensors (capacitive)
- Belt usage detection

**Suppression Logic:**
```
if occupant_weight < 13kg:
    suppress_passenger_airbag()
elif occupant_position == "out_of_position":
    reduce_deployment_force()
```

### 3.2 Seatbelt Systems

#### 3.2.1 Three-Point Seatbelt

**Components:**
1. Webbing (polyester, 46mm width)
2. Retractor with inertial locking
3. Buckle (minimum 20 kN release force)
4. Anchor points (structural attachment)

**Load Limiting:**
- Force limiter: 4-6 kN (standard)
- Adaptive limiting: 3-7 kN (based on occupant)

#### 3.2.2 Pretensioners

**Pyrotechnic Pretensioner:**
- Activation time: 10-15 milliseconds
- Belt retraction: 100-150 mm
- Force: 2-4 kN

**Electric Pretensioner:**
- Activation time: 20-30 milliseconds
- Reversible operation
- Pre-crash activation capability

**Activation Criteria:**
```
if crash_imminent or crash_detected:
    activate_pretensioner()
    remove_belt_slack()
```

### 3.3 Structural Safety

#### 3.3.1 Crumple Zones

**Design Principles:**
1. Progressive deformation
2. Maximum energy absorption
3. Controlled collapse path
4. Minimize intrusion

**Energy Absorption:**
```
E_absorbed = ∫₀ᵈ F(x) dx
```

Where:
- `E` = Kinetic energy to absorb
- `F(x)` = Deformation force at distance x
- `d` = Crumple zone length

**Target Deceleration:**
```
a_avg = v² / (2 × d) ≤ 40 g
```

#### 3.3.2 Passenger Safety Cell

**Requirements:**
- Rigid structure maintaining survival space
- A-pillar intrusion: < 100 mm (frontal crash)
- B-pillar intrusion: < 150 mm (side crash)
- Roof strength: 3× vehicle weight (FMVSS 216)

**Material Specifications:**
- High-strength steel (> 1000 MPa)
- Hot-stamped components (A/B pillars)
- Aluminum space frames (alternative)

#### 3.3.3 Door Beams

**Side Impact Beams:**
- Material: Ultra-high-strength steel
- Minimum strength: 1500 MPa
- Placement: Hip and shoulder height
- Extends full door width

**Performance:**
```
Beam_strength = (F × L) / (4 × δ_max)
```

Where:
- `F` = Applied force
- `L` = Beam length
- `δ_max` = Maximum deflection

---

## 4. Crash Testing Standards

### 4.1 Frontal Impact Tests

#### 4.1.1 Full Frontal Rigid Barrier (FFRB)

**Test Parameters:**
- Impact speed: 48-56 km/h
- Barrier: Rigid, flat, vertical surface
- Vehicle orientation: 0° (perpendicular)

**Instrumentation:**
- Hybrid III 50th percentile male dummies
- Head, chest, femur, tibia acceleration sensors
- Displacement transducers

**Measurement Points:**
```
{
  "head": {
    "HIC-15": < 700,
    "HIC-36": < 1000,
    "peak_acceleration": < 80 g
  },
  "chest": {
    "compression": < 50 mm,
    "acceleration_3ms": < 60 g
  },
  "femur": {
    "load": < 10 kN (each)
  }
}
```

#### 4.1.2 Frontal Offset Deformable Barrier

**Test Setup:**
- Impact speed: 64 km/h (Euro NCAP)
- Overlap: 40% (driver side)
- Barrier: Aluminum honeycomb (EEVC)

**Scoring:**
- Head protection: 0-4 points
- Chest protection: 0-4 points
- Upper leg protection: 0-2 points
- Lower leg protection: 0-2 points

**Good Performance:**
- HIC-15 < 500
- Chest compression < 42 mm
- Femur load < 7.6 kN
- Tibia index < 1.0

### 4.2 Side Impact Tests

#### 4.2.1 Moving Deformable Barrier (MDB)

**Test Configuration:**
- Barrier speed: 50 km/h
- Barrier mass: 950 kg
- Impact height: Ground to 450 mm
- Deformable face: 300mm depth

**Dummies:**
- Driver: EuroSID-2
- Rear passenger: WorldSID or Q6 child dummy

**Injury Criteria:**
```
{
  "head": {
    "HPC": < 1000
  },
  "chest": {
    "rib_deflection": < 42 mm,
    "viscous_criterion": < 1.0
  },
  "abdomen": {
    "force": < 2.5 kN
  },
  "pelvis": {
    "force": < 6.0 kN
  }
}
```

#### 4.2.2 Side Pole Impact

**Test Parameters:**
- Impact speed: 32 km/h
- Pole diameter: 254 mm (10 inches)
- Impact location: Driver's head position

**Head Protection:**
- HPC (Head Protection Criterion) < 1000
- Focuses on curtain airbag performance

### 4.3 Rollover Testing

#### 4.3.1 Roof Strength Test (FMVSS 216a)

**Test Method:**
- Quasi-static loading
- Load plate: 762 × 1829 mm
- Force direction: 5° from vertical, toward rear

**Performance Requirement:**
```
Peak_Force ≥ 3.0 × Vehicle_Weight

or

Strength-to-Weight Ratio (SWR) ≥ 3.0
```

**Measurement:**
- Maximum force before 127mm roof crush
- Both driver and passenger side tested

#### 4.3.2 Dynamic Rollover Test

**Fishhook Maneuver:**
- Initial speed: 50-60 km/h
- Steering input: Sine wave (0.7 Hz)
- Measure: SSF (Static Stability Factor)

**SSF Calculation:**
```
SSF = track_width / (2 × center_of_gravity_height)
```

Rating:
- SSF ≥ 1.45: 5 stars
- SSF 1.20-1.44: 4 stars
- SSF 1.10-1.19: 3 stars
- SSF < 1.10: 1-2 stars

### 4.4 Rear Impact Testing

#### 4.4.1 Whiplash Protection

**Test Setup:**
- Sled test with seat/head restraint
- Acceleration pulse: 16 km/h ΔV
- Dummy: BioRID-II or RID-3D

**Metrics:**
- NIC (Neck Injury Criterion)
- Nkm (Lower neck shear)
- Upper neck tension/compression
- Head restraint position (< 60mm from head)

**Scoring (Euro NCAP):**
- Good: Dynamic and geometric performance optimal
- Adequate: Meets minimum requirements
- Marginal: Poor geometric setup
- Poor: High injury risk

---

## 5. Occupant Protection

### 5.1 Injury Biomechanics

#### 5.1.1 Head Injury Criteria

**HIC-15 (Head Injury Criterion, 15ms window):**
```
HIC = max[(t₂ - t₁) × [1/(t₂-t₁) × ∫(t₁ to t₂) a(t)dt]^2.5]

where: (t₂ - t₁) ≤ 15 ms
```

**Thresholds:**
- HIC-15 < 700: Low injury risk
- HIC-15 700-1000: Moderate risk
- HIC-15 > 1000: High risk

**Brain Injury Prediction:**
```
P(AIS ≥ 3) = 1 / (1 + e^(5.02 - 0.00351×HIC))
```

#### 5.1.2 Chest Injury

**Chest Compression:**
```
Compression = (δ / D) × 100%
```

Where:
- `δ` = Maximum sternum deflection
- `D` = Initial chest depth

**Thresholds:**
- < 20%: Low risk
- 20-30%: Moderate risk
- > 30%: High risk (AIS 3+)

**Viscous Criterion (VC):**
```
VC = max[V(t) × C(t)]
```

Where:
- `V(t)` = Deflection velocity (m/s)
- `C(t)` = Compression ratio
- Threshold: VC < 1.0 m/s

#### 5.1.3 Lower Extremity Injury

**Femur Load:**
- Threshold: 10 kN (each leg)
- Bilateral: 20 kN combined

**Tibia Index (TI):**
```
TI = (M / M_crit)² + (F / F_crit)²
```

Where:
- `M` = Bending moment
- `F` = Axial force
- `M_crit` = 225 Nm
- `F_crit` = 35.9 kN
- Threshold: TI < 1.3

### 5.2 Restraint System Optimization

#### 5.2.1 Multi-Stage Airbag Deployment

**Stage Selection Algorithm:**
```
function selectStage(crash_severity, occupant_class, belt_status):
    if belt_status == "unbuckled":
        return STAGE_2  # Full deployment

    if occupant_class == "small_adult" or "child":
        if crash_severity < THRESHOLD_MED:
            return STAGE_1  # Low power
        else:
            return STAGE_1_5  # Medium power

    if occupant_class == "large_adult":
        if crash_severity > THRESHOLD_HIGH:
            return STAGE_2  # High power
        else:
            return STAGE_1  # Low power
```

#### 5.2.2 Adaptive Load Limiting

**Belt Force Profile:**
```
F_belt(t) = min(F_max, K × δ(t))
```

Where:
- `F_max` = Load limit (varies by occupant)
- `K` = Belt stiffness
- `δ(t)` = Belt extension

**Adaptive Limits:**
- Small occupant (< 50 kg): 3.0 kN
- Medium occupant (50-80 kg): 4.5 kN
- Large occupant (> 80 kg): 6.0 kN

### 5.3 Advanced Protection Systems

#### 5.3.1 Pre-Collision Systems

**Pre-Safe Features:**
1. Belt pretensioning (reversible)
2. Seat position adjustment
3. Window/sunroof closure
4. Headrest positioning

**Activation Timeline:**
```
t = -300ms: Collision detection
t = -200ms: Belt pre-tensioning begins
t = -100ms: Seat/headrest adjustment
t = 0ms: Impact
t = +25ms: Airbag deployment
```

#### 5.3.2 Post-Collision Safety

**Automatic Response:**
1. Hazard lights activation
2. Door unlocking
3. Fuel pump cutoff
4. Battery isolation
5. eCall emergency notification

**eCall Data Transmission:**
```json
{
  "timestamp": "ISO 8601",
  "location": {"lat": 0.0, "lon": 0.0},
  "heading": 0,
  "vin": "vehicle_id",
  "crash_severity": "high|medium|low",
  "airbag_deployment": true,
  "occupants": 2
}
```

---

## 6. Pedestrian Safety

### 6.1 Pedestrian Impact Protection

#### 6.1.1 Test Zones

**Hood/Bonnet Testing:**
- Child head impact (2.5 kg, 6.5 m drop height)
- Adult head impact (4.5 kg, 9.5 m drop height)
- Upper leg impact (11.3 kg, 40 km/h)
- Lower leg impact (13.4 kg, 40 km/h)

**Scoring Grid:**
- Test points spaced 100mm apart
- Wrapped Around Distance (WAD) 1000-2500mm
- Color coding: Green (good), Yellow (adequate), Orange (marginal), Red (poor)

#### 6.1.2 Head Impact Criteria

**HIC Thresholds:**
```
Child head (2.5 kg):
  HIC < 1000: 4 points
  HIC 1000-1350: 2.7 points
  HIC 1350-1700: 1.3 points
  HIC > 1700: 0 points

Adult head (4.5 kg):
  HIC < 1000: 4 points
  HIC 1000-1350: 2.7 points
  HIC 1350-1700: 1.3 points
  HIC > 1700: 0 points
```

#### 6.1.3 Leg Impact Protection

**Upper Leg:**
- Bending moment: < 300 Nm
- Sum of impacts: < 510 Nm
- Force: < 5.0 kN

**Lower Leg:**
- Tibia bending: < 340 Nm
- Tibia shear: < 6.0 kN
- MCL (Medial Collateral Ligament) elongation: < 23 mm
- ACL (Anterior Cruciate Ligament) force: < 4.0 kN

### 6.2 Active Pedestrian Protection

#### 6.2.1 Pedestrian AEB

**Detection System:**
- Forward camera with pedestrian recognition
- Radar (77 GHz) for range/velocity
- Processing: CNN-based classification

**Performance Requirements:**
- Detection range: 60 meters
- Classification accuracy: > 95%
- Reaction time: < 150 ms
- Operating speed: 20-60 km/h

**Test Scenarios (Euro NCAP):**
1. Adult crossing (25%, 50%, 75% travel)
2. Child darting from behind obstruction
3. Adult walking along road (back turned)
4. Child crossing from behind parked vehicles

#### 6.2.2 Active Hood Systems

**Pop-Up Hood:**
- Actuation time: 30-50 milliseconds
- Lift height: 80-100 mm
- Activation speed: > 25 km/h
- Pedestrian detection required

**Mechanism:**
```
if pedestrian_impact_detected():
    fire_hood_actuators()  # Pyrotechnic or spring
    lift_rear_hood(80mm)
    increase_impact_zone_depth()
```

### 6.3 External Airbags

**Windshield Airbag:**
- Deployment: 20-30 milliseconds
- Coverage: 50% of windshield
- Thickness: 80-100 mm when inflated

**Benefits:**
- Reduces head HIC by 30-40%
- Protects against A-pillar impact
- Mitigates windshield penetration

---

## 7. Child Safety Systems

### 7.1 Child Restraint Standards

#### 7.1.1 Age/Weight Groups

**Group Classification:**
- **Group 0**: 0-10 kg (0-9 months) - Rear-facing
- **Group 0+**: 0-13 kg (0-15 months) - Rear-facing
- **Group I**: 9-18 kg (9 months-4 years) - Forward/rear-facing
- **Group II**: 15-25 kg (4-6 years) - Booster
- **Group III**: 22-36 kg (6-12 years) - Booster

#### 7.1.2 ISOFIX Attachment

**Specifications (ISO 13216):**
- Lower anchorages: 280 ± 10 mm spacing
- Location: Seat bight (junction of seat cushion and backrest)
- Strength: 800 kg force (8 kN)
- Top tether: 0-2000 mm from ISOFIX points

**Connector Types:**
- Rigid ISOFIX: Fixed arms
- Flexible ISOFIX: Strap/belt connectors
- Semi-universal: With top tether or support leg

**Installation Verification:**
```
function verifyISOFIX():
    check_connector_engagement()  # Visual/audible indicator
    check_support_leg_contact()   # Floor contact
    check_tether_tension()        # Minimum slack
    return all_checks_pass
```

#### 7.1.3 LATCH System (North America)

**Lower Anchorages and Tethers for Children:**
- Two lower anchorages per seating position
- Weight limit: 65 pounds (child + seat)
- Top tether: All forward-facing seats

### 7.2 Child Crash Testing

#### 7.2.1 Test Dummies

**Dummy Specifications:**
- **P0**: Newborn (3.4 kg)
- **P3/4**: 9-month-old (9 kg)
- **P6**: 3-year-old (22 kg)
- **Q6**: 6-year-old (23 kg)
- **Q10**: 10-year-old (33 kg)

#### 7.2.2 Injury Criteria (Children)

**Head:**
```
HIC-15 thresholds (age-dependent):
  0-18 months: HIC < 600
  18-36 months: HIC < 650
  3-6 years: HIC < 700
  6+ years: HIC < 700
```

**Chest:**
- Acceleration: < 55 g (3ms clip)
- Deflection: < 40 mm (P6, Q6)

**Neck:**
```
Tension:
  P3/4: < 1.2 kN
  P6: < 1.7 kN
  Q6: < 2.0 kN
  Q10: < 2.3 kN

Compression:
  All: < 1.0 kN
```

### 7.3 Child Airbag Safety

#### 7.3.1 Passenger Airbag Suppression

**Detection Methods:**
1. Weight-based (seat mat sensor)
2. Capacitive occupancy detection
3. RFID child seat recognition

**Suppression Logic:**
```
function shouldSuppressAirbag():
    if occupant_weight < 13.6 kg:  # 30 lbs
        return SUPPRESS

    if rearfacing_child_seat_detected():
        return SUPPRESS

    if occupant_classification == "CHILD":
        return SUPPRESS

    return DEPLOY_NORMAL
```

**Requirements:**
- Clear indicator to driver (yellow LED)
- Warning label on sun visor
- Must not suppress for adults (> 35 kg)

---

## 8. Data Formats

### 8.1 Crash Data Recording

#### 8.1.1 Event Data Recorder (EDR)

**Pre-Crash Data (5 seconds before):**
```json
{
  "pre_crash": {
    "duration": 5,
    "sample_rate": 10,
    "data": [
      {
        "time": -5.0,
        "speed": 65.3,
        "throttle": 45,
        "brake": 0,
        "steering": -12,
        "abs": false,
        "esc": false
      }
    ]
  }
}
```

**Crash Event Data:**
```json
{
  "crash_event": {
    "timestamp": "2025-01-15T14:23:45Z",
    "duration": 0.3,
    "sample_rate": 1000,
    "delta_v": {
      "longitudinal": -45.3,
      "lateral": 12.1,
      "resultant": 47.1
    },
    "peak_acceleration": {
      "x": -38.5,
      "y": 8.2,
      "z": -2.1
    },
    "airbag_deployment": {
      "driver_frontal": {"time": 0.028, "stage": 2},
      "passenger_frontal": {"time": 0.032, "stage": 2},
      "driver_side": {"time": null},
      "curtain_left": {"time": 0.018}
    },
    "belt_pretensioner": {
      "driver": {"time": 0.012},
      "passenger": {"time": 0.012}
    },
    "multiple_events": false
  }
}
```

#### 8.1.2 Safety System Status

```json
{
  "safety_status": {
    "timestamp": "2025-01-15T14:23:40Z",
    "vehicle_id": "WDB1234567890",
    "systems": {
      "abs": {
        "status": "operational",
        "last_test": "2025-01-10",
        "fault_codes": []
      },
      "esc": {
        "status": "operational",
        "calibration_date": "2024-12-15"
      },
      "airbags": {
        "driver": {"status": "armed", "deployment_count": 0},
        "passenger": {"status": "suppressed", "reason": "child_detected"}
      },
      "aeb": {
        "status": "active",
        "sensitivity": "normal",
        "range": 145.3
      },
      "ldw": {
        "status": "active",
        "last_calibration": "2024-11-20"
      }
    }
  }
}
```

### 8.2 Safety Rating Data

#### 8.2.1 NCAP Star Rating

```json
{
  "ncap_rating": {
    "program": "Euro NCAP",
    "year": 2025,
    "vehicle": {
      "make": "Example",
      "model": "SafetyCar",
      "variant": "Premium"
    },
    "overall_rating": 5,
    "scores": {
      "adult_occupant": {
        "percentage": 92,
        "points": 35.0,
        "max_points": 38.0,
        "frontal_offset": 15.2,
        "frontal_full_width": 7.8,
        "side_barrier": 7.9,
        "side_pole": 4.1
      },
      "child_occupant": {
        "percentage": 87,
        "points": 42.6,
        "max_points": 49.0,
        "dynamic": 24.0,
        "vehicle_assessment": 13.0,
        "child_presence_detection": 5.6
      },
      "vulnerable_road_users": {
        "percentage": 78,
        "points": 42.1,
        "max_points": 54.0,
        "pedestrian": 18.5,
        "cyclist": 8.9,
        "aeb_pedestrian": 14.7
      },
      "safety_assist": {
        "percentage": 85,
        "points": 15.3,
        "max_points": 18.0,
        "aeb_car": 7.0,
        "lane_support": 4.3,
        "speed_assistance": 4.0
      }
    }
  }
}
```

---

## 9. API Interface

### 9.1 Safety Assessment API

#### 9.1.1 Crash Safety Evaluation

```typescript
interface CrashAssessmentRequest {
  vehicleMass: number;           // kg
  impactVelocity: number;        // m/s
  impactAngle: number;           // degrees (0=frontal, 90=side)
  crumpleZoneLength: number;     // meters
  occupants: OccupantInfo[];
}

interface OccupantInfo {
  position: 'driver' | 'passenger' | 'rear_left' | 'rear_right' | 'rear_center';
  mass: number;                  // kg
  height: number;                // meters
  age: number;                   // years
  beltStatus: boolean;
  airbagEnabled: boolean;
}

interface CrashAssessmentResponse {
  impactForce: number;           // Newtons
  deceleration: number;          // g's
  impactDuration: number;        // seconds
  energyAbsorbed: number;        // joules
  occupantInjury: {
    [position: string]: {
      hic15: number;
      chestDeflection: number;   // mm
      femurLoad: number;         // kN
      tibiaIndex: number;
      injuryRisk: 'low' | 'moderate' | 'high' | 'severe';
      ais: number;               // Abbreviated Injury Scale
    }
  };
  structuralIntegrity: {
    cabinIntrusion: number;      // mm
    doorOpenability: boolean;
    pedalDisplacement: number;   // mm
  };
  safetyRating: number;          // 0-5 stars
}
```

#### 9.1.2 Airbag Deployment Decision

```typescript
interface AirbagDeploymentRequest {
  crashSeverity: number;         // 0-10 scale
  crashType: 'frontal' | 'side' | 'rear' | 'rollover';
  impactVelocity: number;        // m/s
  occupantPresence: boolean[];   // [driver, passenger, rear_L, rear_R]
  occupantClassification: string[]; // ['adult', 'child', 'infant', 'empty']
  beltStatus: boolean[];
  crashPulse: number[];          // acceleration time series (g)
}

interface AirbagDeploymentResponse {
  shouldDeploy: {
    driverFrontal: {deploy: boolean, stage: 0 | 1 | 2, timing: number},
    passengerFrontal: {deploy: boolean, stage: 0 | 1 | 2, timing: number},
    driverSide: {deploy: boolean, timing: number},
    passengerSide: {deploy: boolean, timing: number},
    curtainLeft: {deploy: boolean, timing: number},
    curtainRight: {deploy: boolean, timing: number}
  };
  pretensioners: {
    driver: {activate: boolean, timing: number, force: number},
    passenger: {activate: boolean, timing: number, force: number}
  };
  reasoning: string[];
}
```

### 9.2 Active Safety API

#### 9.2.1 Collision Warning

```typescript
interface CollisionWarningRequest {
  ownVehicle: {
    velocity: number;            // m/s
    position: {x: number, y: number};
    heading: number;             // degrees
  };
  obstacles: Array<{
    type: 'vehicle' | 'pedestrian' | 'cyclist' | 'object';
    position: {x: number, y: number};
    velocity: {vx: number, vy: number};
    dimensions: {length: number, width: number};
    confidence: number;          // 0-1
  }>;
  roadCondition: 'dry' | 'wet' | 'snow' | 'ice';
}

interface CollisionWarningResponse {
  threats: Array<{
    obstacleId: number;
    ttc: number;                 // time to collision (seconds)
    severity: 'low' | 'medium' | 'high' | 'critical';
    collisionProbability: number; // 0-1
    recommendedAction: 'none' | 'warning' | 'brake' | 'emergency_brake' | 'evade';
    evasionPath?: {x: number, y: number}[];
  }>;
  warningLevel: 0 | 1 | 2 | 3;
  interventionRequired: boolean;
}
```

---

## 10. Safety Protocols

### 10.1 ISO 26262 - Functional Safety

#### 10.1.1 ASIL Classification

**Automotive Safety Integrity Levels:**

| ASIL | Severity | Exposure | Controllability | Example Systems |
|------|----------|----------|-----------------|-----------------|
| QM | No injury | - | - | Windshield wipers |
| ASIL-A | Light injuries | Low | Usually | Rear lights |
| ASIL-B | Moderate injuries | Medium | Normally | ESC |
| ASIL-C | Severe injuries | High | Difficult | AEB |
| ASIL-D | Life-threatening | Very high | Unlikely | Airbag control |

**Classification Formula:**
```
ASIL = f(Severity, Exposure, Controllability)

Severity (S):
  S0: No injuries
  S1: Light injuries
  S2: Severe injuries
  S3: Life-threatening/fatal

Exposure (E):
  E0: Incredible (< 0.001%)
  E1: Very low (< 0.1%)
  E2: Low (< 1%)
  E3: Medium (< 10%)
  E4: High (≥ 10%)

Controllability (C):
  C0: Controllable in general
  C1: Simply controllable
  C2: Normally controllable
  C3: Difficult to control
```

#### 10.1.2 Safety Requirements

**Airbag System (ASIL-D):**
1. Dual microcontroller architecture
2. Watchdog timers (independent)
3. Memory protection (ECC RAM)
4. Diagnostic coverage > 99%
5. Single point fault metric > 99%
6. Latent fault metric > 90%

**Development Process:**
```
1. Hazard Analysis and Risk Assessment
2. Safety Goals definition
3. Functional Safety Concept
4. Technical Safety Concept
5. Hardware/Software Safety Requirements
6. Implementation
7. Verification and Validation
8. Functional Safety Assessment
```

### 10.2 FMVSS Compliance (USA)

#### 10.2.1 Key Standards

**FMVSS 126 - Electronic Stability Control:**
- Mandatory since 2012
- Sine-with-dwell test
- Lateral displacement limits

**FMVSS 208 - Occupant Protection:**
- Frontal crash protection
- Dummy injury criteria
- Airbag specifications
- Belted and unbelted tests

**FMVSS 214 - Side Impact Protection:**
- Moving deformable barrier test
- Side impact dummies
- Door beam requirements

**FMVSS 216a - Roof Crush Resistance:**
- Strength-to-weight ratio ≥ 3.0
- Both sides tested

#### 10.2.2 Compliance Testing

**Test Matrix:**
```
{
  "FMVSS_208": {
    "tests": [
      "35 mph rigid barrier (belted)",
      "25 mph rigid barrier (unbelted)",
      "25 mph angular barrier (belted)"
    ],
    "dummies": ["Hybrid III 50th male", "5th female", "SID-IIs"],
    "measurements": ["HIC", "chest_g", "femur_load"]
  },
  "FMVSS_214": {
    "tests": [
      "33.5 mph MDB (driver side)",
      "33.5 mph MDB (passenger side)",
      "20 mph pole impact"
    ],
    "dummies": ["SID-IIs", "WorldSID"],
    "measurements": ["TTI", "pelvic_force", "abdominal_force"]
  }
}
```

### 10.3 Euro NCAP Protocol

#### 10.3.1 Rating Methodology (2025)

**Overall Rating Calculation:**
```
Adult Occupant: 40% weight
Child Occupant: 20% weight
Vulnerable Road Users: 20% weight
Safety Assist: 20% weight

Minimum thresholds for 5 stars:
  Adult Occupant: ≥ 80%
  Child Occupant: ≥ 80%
  VRU: ≥ 70%
  Safety Assist: ≥ 70%
```

**Points Distribution:**
```json
{
  "adult_occupant": {
    "frontal_impact": 16,
    "side_impact": 16,
    "far_side": 4,
    "rescue": 2,
    "total": 38
  },
  "child_occupant": {
    "dynamic_tests": 24,
    "vehicle_assessment": 13,
    "CRS_installation": 12,
    "total": 49
  },
  "vru": {
    "head_impact": 18,
    "pelvis_impact": 6,
    "leg_impact": 6,
    "aeb_pedestrian": 9,
    "aeb_cyclist": 9,
    "aeb_motorcyclist": 6,
    "total": 54
  },
  "safety_assist": {
    "aeb_car": 7,
    "lane_support": 5,
    "speed_assistance": 4,
    "driver_monitoring": 2,
    "total": 18
  }
}
```

#### 10.3.2 Modifiers and Penalties

**Bonuses:**
- Advanced restraints: +1.0 point
- Rescue systems: +0.5 point
- Post-crash eCall: +0.5 point

**Penalties:**
- No seatbelt reminder: -1.0 star
- Poor geometry (whiplash): -0.5 point
- Limited safety pack availability: -1.0 star

---

## 11. References

### 11.1 Standards and Regulations

1. **ISO 26262**: Road vehicles - Functional safety
2. **ECE R94**: Frontal impact protection
3. **ECE R95**: Side impact protection
4. **ECE R16**: Seatbelts and restraints
5. **FMVSS 208**: Occupant crash protection
6. **FMVSS 214**: Side impact protection
7. **FMVSS 226**: Ejection mitigation
8. **Euro NCAP**: Assessment Protocol 2025
9. **IIHS**: Test and Rating Protocols
10. **ISO 13216**: ISOFIX child restraint systems

### 11.2 Scientific References

1. Mertz, H.J. (1993). "Anthropomorphic Test Devices"
2. Eppinger, R. et al. (1999). "Development of Improved Injury Criteria"
3. Viano, D.C. (2003). "Biomechanics of Brain and Spinal Injuries"
4. Yoganandan, N. (2014). "Injury Biomechanics Research"
5. Gabler, H.C. (2020). "The Future of Automotive Safety"

### 11.3 Crash Test Databases

- **NHTSA**: National crash test database (USA)
- **Euro NCAP**: European test results
- **IIHS**: Insurance institute crash tests
- **JNCAP**: Japan new car assessment
- **ANCAP**: Australasian NCAP
- **Latin NCAP**: Latin America NCAP

### 11.4 Biomechanical Limits

**Summary Table:**

| Body Region | Metric | Threshold | ASIL |
|-------------|--------|-----------|------|
| Head | HIC-15 | < 700 | D |
| Brain | Rotational acceleration | < 4500 rad/s² | D |
| Neck | Tension | < 3.3 kN | D |
| Neck | Compression | < 4.0 kN | D |
| Chest | Deflection | < 50 mm | D |
| Chest | VC | < 1.0 m/s | D |
| Abdomen | Force | < 2.5 kN | C |
| Pelvis | Force | < 6.0 kN | C |
| Femur | Axial load | < 10 kN | C |
| Knee | Displacement | < 15 mm | B |
| Tibia | Index | < 1.3 | C |

### 11.5 WIA Standards Integration

- **WIA-AUTO-001**: V2V communication for collision avoidance
- **WIA-AUTO-005**: Autonomous vehicle safety protocols
- **WIA-AUTO-012**: Vehicle sensors and perception
- **WIA-AUTO-015**: OBD and vehicle diagnostics
- **WIA-INTENT**: Driver intent recognition
- **WIA-OMNI-API**: Universal vehicle API gateway
- **WIA-SOCIAL**: Emergency response coordination

---

## Appendix A: Calculation Examples

### A.1 Frontal Crash Energy

```
Given:
- Vehicle mass: 1500 kg
- Impact velocity: 56 km/h = 15.6 m/s
- Crumple zone: 0.8 meters

Kinetic Energy:
E = 1/2 × m × v²
E = 0.5 × 1500 × 15.6²
E = 182,520 joules (182.5 kJ)

Average Deceleration:
a = v² / (2 × d)
a = 15.6² / (2 × 0.8)
a = 152 / 1.6
a = 95 m/s² ≈ 9.7 g

Impact Duration:
t = v / a
t = 15.6 / 95
t = 0.164 seconds (164 ms)

Average Force:
F = m × a
F = 1500 × 95
F = 142,500 N (142.5 kN)
```

### A.2 Side Impact Analysis

```
Given:
- Barrier mass: 950 kg
- Barrier velocity: 50 km/h = 13.9 m/s
- Vehicle mass: 1500 kg
- Door intrusion limit: 150 mm

Momentum Transfer:
p = m₁ × v₁
p = 950 × 13.9
p = 13,205 kg⋅m/s

Vehicle Velocity After Impact:
v₂ = p / (m₁ + m₂)
v₂ = 13,205 / (950 + 1500)
v₂ = 5.4 m/s

Energy Absorbed by Vehicle:
E = 1/2 × m₂ × v₂²
E = 0.5 × 1500 × 5.4²
E = 21,870 joules (21.9 kJ)

Required Door Beam Strength:
F_avg = E / d
F_avg = 21,870 / 0.15
F_avg = 145,800 N (145.8 kN)
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-022 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
