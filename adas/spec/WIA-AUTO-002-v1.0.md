# WIA-AUTO-002: ADAS - Advanced Driver Assistance System Specification v1.0

> **Standard ID:** WIA-AUTO-002
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Safety Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Sensor Technologies](#2-sensor-technologies)
3. [Sensor Fusion Algorithms](#3-sensor-fusion-algorithms)
4. [Object Detection and Classification](#4-object-detection-and-classification)
5. [Lane Detection and Keeping](#5-lane-detection-and-keeping)
6. [Collision Avoidance Systems](#6-collision-avoidance-systems)
7. [Adaptive Cruise Control](#7-adaptive-cruise-control)
8. [SAE Automation Levels](#8-sae-automation-levels)
9. [Data Formats and Protocols](#9-data-formats-and-protocols)
10. [Safety Protocols and Validation](#10-safety-protocols-and-validation)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for Advanced Driver Assistance Systems (ADAS), providing standardized interfaces, algorithms, and safety protocols for intelligent vehicle assistance technologies.

### 1.2 Scope

The standard covers:
- Sensor hardware specifications and integration
- Sensor fusion algorithms for multi-modal data processing
- Object detection, classification, and tracking
- Lane detection and lane keeping assistance
- Collision prediction and avoidance
- Adaptive cruise control systems
- SAE Level 0-5 automation support
- Safety validation and testing protocols

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to save lives by reducing traffic accidents through intelligent vehicle assistance. According to WHO, road traffic crashes kill 1.35 million people annually. ADAS technology has the potential to prevent up to 90% of these accidents.

### 1.4 Terminology

- **ADAS**: Advanced Driver Assistance System
- **LiDAR**: Light Detection and Ranging
- **TTC**: Time to Collision
- **ACC**: Adaptive Cruise Control
- **LKA**: Lane Keeping Assist
- **AEB**: Automatic Emergency Braking
- **V2V**: Vehicle-to-Vehicle communication
- **V2X**: Vehicle-to-Everything communication
- **SAE**: Society of Automotive Engineers
- **ODD**: Operational Design Domain

---

## 2. Sensor Technologies

### 2.1 LiDAR (Light Detection and Ranging)

#### 2.1.1 Specifications

```
Measurement Principle: Time-of-Flight (ToF)
Wavelength: 905 nm or 1550 nm
Range: 200 meters (typical)
Accuracy: ±2-5 cm
Resolution: 0.1° - 0.2° (angular)
Frame Rate: 10-20 Hz
Field of View: 360° (rotating) or 120° (solid-state)
```

#### 2.1.2 Point Cloud Format

```
Point = (x, y, z, intensity, timestamp)
```

Where:
- `x, y, z`: 3D coordinates in meters
- `intensity`: Reflection intensity (0-255)
- `timestamp`: Capture time in microseconds

#### 2.1.3 Weather Impact

| Condition | Impact | Mitigation |
|-----------|--------|------------|
| Rain | 30-50% range reduction | Increase sensor cleaning frequency |
| Fog | 50-70% range reduction | Switch to radar-primary mode |
| Snow | 40-60% range reduction | Combine with camera validation |
| Direct Sunlight | Minimal | Use appropriate wavelength (1550nm) |

### 2.2 Radar (Radio Detection and Ranging)

#### 2.2.1 Specifications

```
Frequency: 77 GHz (automotive radar band)
Range:
  - Long-range: 150-200 meters
  - Medium-range: 50-100 meters
  - Short-range: 0.2-30 meters
Accuracy: ±0.1-0.5 meters
Velocity Accuracy: ±0.1 m/s (Doppler)
Field of View:
  - Long-range: ±10°
  - Medium-range: ±45°
  - Short-range: ±75°
Update Rate: 10-20 Hz
```

#### 2.2.2 Detection Format

```
Target = {
  range: number,        // meters
  azimuth: number,      // degrees
  elevation: number,    // degrees
  velocity: number,     // m/s (radial)
  rcs: number,          // Radar Cross Section (dBsm)
  snr: number           // Signal-to-Noise Ratio (dB)
}
```

#### 2.2.3 Advantages

- **All-weather operation**: Minimal impact from rain, fog, snow
- **Velocity measurement**: Direct Doppler measurement
- **Long-range**: Effective at highway speeds
- **Penetration**: Can detect through plastic, fog, dust

### 2.3 Camera (Vision Systems)

#### 2.3.1 Specifications

```
Resolution: 1920×1080 (Full HD) minimum
Frame Rate: 30-60 FPS
Dynamic Range: >100 dB (HDR)
Field of View: 50°-120° (multiple cameras)
Color Depth: 12-bit minimum
Shutter: Global shutter preferred (eliminates rolling shutter artifacts)
```

#### 2.3.2 Camera Types

| Type | Purpose | Mounting | FOV |
|------|---------|----------|-----|
| Front Wide | Lane detection, signs, lights | Windshield (center) | 120° |
| Front Narrow | Long-range object detection | Windshield (center) | 50° |
| Side | Blind spot monitoring | Side mirrors | 90° |
| Rear | Parking, backup | Rear bumper/license plate | 130° |
| Interior | Driver monitoring | Dashboard/mirror | 90° |

#### 2.3.3 Image Processing Pipeline

```
1. Image Acquisition → Raw sensor data
2. Pre-processing → Noise reduction, distortion correction
3. Feature Extraction → Edge detection, HOG, SIFT
4. Object Detection → YOLO, SSD, R-CNN
5. Classification → CNN-based classification
6. Tracking → Kalman filter, Hungarian algorithm
7. Post-processing → Filtering, validation
```

### 2.4 Ultrasonic Sensors

#### 2.4.1 Specifications

```
Frequency: 40-58 kHz
Range: 0.15-5 meters
Accuracy: ±1-3 cm
Beam Width: 60°-120°
Update Rate: 10-15 Hz
Operating Temperature: -40°C to +85°C
```

#### 2.4.2 Applications

- Parking assistance (close-range)
- Low-speed maneuvering
- Blind spot detection (supplement)
- Cross-traffic alert

---

## 3. Sensor Fusion Algorithms

### 3.1 Bayesian Sensor Fusion

The foundation of multi-sensor integration using Bayes' theorem:

```
P(X|Z₁,Z₂,...,Zₙ) = η × P(X) × ∏ᵢ P(Zᵢ|X)
```

Where:
- `P(X|Z₁,Z₂,...,Zₙ)` = Posterior probability of state X given all measurements
- `P(X)` = Prior probability (prediction)
- `P(Zᵢ|X)` = Likelihood of measurement Zᵢ given state X
- `η` = Normalization constant

### 3.2 Kalman Filter

For linear systems with Gaussian noise:

#### 3.2.1 Prediction Step

```
x̂ₖ₊₁|ₖ = F × x̂ₖ|ₖ + B × uₖ
Pₖ₊₁|ₖ = F × Pₖ|ₖ × Fᵀ + Q
```

Where:
- `x̂ₖ₊₁|ₖ` = Predicted state
- `F` = State transition matrix
- `B` = Control input matrix
- `uₖ` = Control vector
- `Pₖ₊₁|ₖ` = Predicted covariance
- `Q` = Process noise covariance

#### 3.2.2 Update Step

```
Kₖ = Pₖ|ₖ₋₁ × Hᵀ × (H × Pₖ|ₖ₋₁ × Hᵀ + R)⁻¹
x̂ₖ|ₖ = x̂ₖ|ₖ₋₁ + Kₖ × (zₖ - H × x̂ₖ|ₖ₋₁)
Pₖ|ₖ = (I - Kₖ × H) × Pₖ|ₖ₋₁
```

Where:
- `Kₖ` = Kalman gain
- `H` = Measurement matrix
- `zₖ` = Measurement vector
- `R` = Measurement noise covariance

### 3.3 Extended Kalman Filter (EKF)

For non-linear systems:

```
x̂ₖ₊₁|ₖ = f(x̂ₖ|ₖ, uₖ)
Pₖ₊₁|ₖ = Fⱼ × Pₖ|ₖ × Fⱼᵀ + Q

Fⱼ = ∂f/∂x |ₓ₌ₓ̂ₖ|ₖ  (Jacobian of f)
```

### 3.4 Particle Filter

For highly non-linear, non-Gaussian systems:

```
1. Initialize N particles: {xᵢ⁽⁰⁾, wᵢ⁽⁰⁾}
2. Predict: xᵢ⁽ᵏ⁾ ~ p(x|xᵢ⁽ᵏ⁻¹⁾, u)
3. Update weights: wᵢ⁽ᵏ⁾ = wᵢ⁽ᵏ⁻¹⁾ × p(z|xᵢ⁽ᵏ⁾)
4. Normalize: wᵢ⁽ᵏ⁾ = wᵢ⁽ᵏ⁾ / Σⱼ wⱼ⁽ᵏ⁾
5. Resample if Nₑff < threshold
```

### 3.5 Sensor Fusion Architecture

```
┌─────────────────────────────────────────────────────┐
│                 Perception Layer                     │
├──────────┬──────────┬──────────┬──────────┬─────────┤
│  LiDAR   │  Radar   │  Camera  │Ultrasonic│   GPS   │
│ Detector │ Detector │ Detector │ Detector │         │
└────┬─────┴────┬─────┴────┬─────┴────┬─────┴────┬────┘
     │          │          │          │          │
     └──────────┴──────────┴──────────┴──────────┘
                         │
              ┌──────────▼──────────┐
              │   Data Association   │
              │  (Hungarian Method)  │
              └──────────┬──────────┘
                         │
              ┌──────────▼──────────┐
              │   Kalman Filtering   │
              │   (State Estimation) │
              └──────────┬──────────┘
                         │
              ┌──────────▼──────────┐
              │  Object Tracking     │
              │  (Multi-target)      │
              └──────────┬──────────┘
                         │
              ┌──────────▼──────────┐
              │  Fused Object List   │
              └─────────────────────┘
```

---

## 4. Object Detection and Classification

### 4.1 Object Categories

Standard object classes for ADAS:

| Class ID | Name | Priority | Typical Size (m) |
|----------|------|----------|------------------|
| 0 | Unknown | Low | Variable |
| 1 | Car | High | 4.5 × 1.8 × 1.5 |
| 2 | Truck | High | 12 × 2.5 × 4.0 |
| 3 | Bus | High | 12 × 2.5 × 3.5 |
| 4 | Motorcycle | High | 2.0 × 0.8 × 1.2 |
| 5 | Bicycle | High | 1.8 × 0.6 × 1.2 |
| 6 | Pedestrian | Critical | 0.5 × 0.5 × 1.7 |
| 7 | Animal | Medium | Variable |
| 8 | Traffic Sign | Medium | 0.6 × 0.6 × 2.5 |
| 9 | Traffic Light | High | 0.3 × 0.3 × 3.0 |
| 10 | Road Barrier | Medium | Variable |

### 4.2 Object Detection Pipeline

#### 4.2.1 LiDAR-based Detection

```
1. Point Cloud Pre-processing
   - Ground removal (RANSAC)
   - ROI filtering
   - Voxelization

2. Clustering
   - DBSCAN or Euclidean clustering
   - Minimum points: 10-50 per cluster

3. Bounding Box Fitting
   - Oriented bounding box (OBB)
   - L-shape fitting for vehicles

4. Feature Extraction
   - Size: length, width, height
   - Shape: aspect ratio, compactness
   - Density: points per volume

5. Classification
   - SVM, Random Forest, or CNN
   - Input: geometric features + intensity
```

#### 4.2.2 Camera-based Detection

Using deep learning (YOLO, SSD, or Faster R-CNN):

```
Input Image → CNN Backbone → Detection Head → NMS → Output
(1920×1080)    (ResNet-50)    (Anchors)      (IoU>0.5)  (Bboxes + Classes)
```

**Performance Requirements:**
- Inference time: < 50ms (for 30 FPS)
- mAP (mean Average Precision): > 90%
- False positive rate: < 1%
- Detection range: > 100m (vehicles)

#### 4.2.3 Radar-based Detection

```
1. Signal Processing
   - FFT (Fast Fourier Transform)
   - CFAR (Constant False Alarm Rate) detection

2. Peak Detection
   - Range-Doppler map
   - Angle estimation (MUSIC or beamforming)

3. Tracking
   - Kalman filter per target
   - Track-before-detect for weak targets

4. Classification
   - RCS-based (Radar Cross Section)
   - Micro-Doppler signature
```

### 4.3 Object Tracking

#### 4.3.1 Single Object Tracking

State vector:
```
x = [px, py, vx, vy, ax, ay, length, width, heading]ᵀ
```

Where:
- `px, py` = Position (m)
- `vx, vy` = Velocity (m/s)
- `ax, ay` = Acceleration (m/s²)
- `length, width` = Object dimensions (m)
- `heading` = Orientation (radians)

#### 4.3.2 Multi-Object Tracking

**Global Nearest Neighbor (GNN):**
```
Cost Matrix C[i,j] = distance(track_i, detection_j)
Assignment = Hungarian_Algorithm(C)
```

**Joint Probabilistic Data Association (JPDA):**
```
βᵢⱼ = probability that detection j originated from track i
x̂ᵢ = Σⱼ βᵢⱼ × x̂ᵢⱼ  (weighted average of all associations)
```

### 4.4 Object State Format

```json
{
  "object_id": 12345,
  "class": "car",
  "class_confidence": 0.97,
  "position": {
    "x": 25.3,
    "y": -2.1,
    "z": 0.0
  },
  "velocity": {
    "x": 22.5,
    "y": 0.2,
    "z": 0.0
  },
  "acceleration": {
    "x": -0.5,
    "y": 0.0,
    "z": 0.0
  },
  "dimensions": {
    "length": 4.5,
    "width": 1.8,
    "height": 1.5
  },
  "heading": 0.05,
  "tracking_age": 2.5,
  "prediction_horizon": 5.0
}
```

---

## 5. Lane Detection and Keeping

### 5.1 Lane Detection Algorithms

#### 5.1.1 Classical Approach

**Canny Edge Detection + Hough Transform:**

```
1. Pre-processing
   - Gaussian blur (5×5 kernel)
   - ROI masking (trapezoid)

2. Edge Detection
   - Canny edges (threshold: 50-150)

3. Line Detection
   - Hough Transform
   - θ ∈ [20°, 80°] ∪ [100°, 160°]
   - ρ resolution: 1 pixel
   - θ resolution: 1°

4. Lane Fitting
   - Linear regression or polynomial fit
   - Degree 2-3 polynomial for curves
```

#### 5.1.2 Deep Learning Approach

**Segmentation Network (U-Net, ENet):**

```
Input Image (1920×1080)
    ↓
Encoder (downsample to 60×34)
    ↓
Decoder (upsample to 1920×1080)
    ↓
Per-pixel Classification
    ↓
Lane Mask
```

**Output:** Pixel-wise lane mask with classes:
- Background
- Ego lane left
- Ego lane right
- Adjacent lane left
- Adjacent lane right

#### 5.1.3 Lane Model

**Polynomial representation:**
```
x(y) = a₀ + a₁y + a₂y² + a₃y³
```

**Clothoid (Euler spiral) representation:**
```
x(s) = ∫₀ˢ cos(θ(t)) dt
y(s) = ∫₀ˢ sin(θ(t)) dt
θ(s) = c₀ + c₁s + c₂s²
```

Where:
- `c₀` = Initial heading
- `c₁` = Curvature
- `c₂` = Curvature rate
- `s` = Arc length

### 5.2 Lane Keeping Assist (LKA)

#### 5.2.1 Lateral Position Control

**PID Controller:**
```
δ = Kₚ × e + Kᵢ × ∫e dt + Kd × de/dt
```

Where:
- `δ` = Steering angle (radians)
- `e` = Lateral offset from lane center (m)
- `Kₚ` = Proportional gain (typical: 0.5-2.0)
- `Kᵢ` = Integral gain (typical: 0.01-0.1)
- `Kd` = Derivative gain (typical: 0.1-0.5)

**Model Predictive Control (MPC):**
```
Minimize: J = Σᵢ (qₑ × eᵢ² + qₑ̇ × ėᵢ² + qδ × δᵢ² + qδ̇ × Δδᵢ²)
Subject to:
  - |δ| ≤ δₘₐₓ
  - |Δδ| ≤ Δδₘₐₓ
  - Vehicle dynamics constraints
```

#### 5.2.2 Lane Departure Warning (LDW)

**Time-to-Lane-Crossing (TLC):**
```
TLC = d_lane_edge / (v × sin(θ))
```

Where:
- `d_lane_edge` = Distance to lane boundary (m)
- `v` = Vehicle velocity (m/s)
- `θ` = Heading angle relative to lane (radians)

**Warning Levels:**
- `TLC > 2.0s`: No warning
- `1.0s < TLC ≤ 2.0s`: Visual warning
- `0.5s < TLC ≤ 1.0s`: Haptic warning (steering vibration)
- `TLC ≤ 0.5s`: Audible warning + corrective steering

### 5.3 Lane Change Assist (LCA)

#### 5.3.1 Safe Lane Change Conditions

```
1. Target lane is clear:
   - No vehicle within safe distance (front and rear)
   - d_front > v × t_gap + d_min
   - d_rear > v_rear × t_gap + d_min

2. Sufficient lateral space:
   - Lane width > vehicle width + margin
   - Margin ≥ 0.5m per side

3. Turn signal activated:
   - Duration ≥ 3 seconds before maneuver

4. Road conditions permit:
   - No construction zone
   - No solid lane markings
   - Weather conditions acceptable
```

#### 5.3.2 Lane Change Trajectory

**Cosine-based trajectory:**
```
y(x) = (w/2) × (1 - cos(πx/L))
```

Where:
- `w` = Lane width (m)
- `L` = Lane change length (m)
- `x` = Longitudinal distance (m)
- `y` = Lateral position (m)

**Duration:**
```
T = L / v
```

Typical values:
- `L = 30-50m` at highway speeds
- `T = 3-5 seconds`

---

## 6. Collision Avoidance Systems

### 6.1 Time-to-Collision (TTC)

#### 6.1.1 Basic TTC Formula

```
TTC = (d - d_safe) / (v_ego - v_target)
```

Where:
- `TTC` = Time to collision (seconds)
- `d` = Current distance (m)
- `d_safe` = Safe stopping distance (m)
- `v_ego` = Ego vehicle speed (m/s)
- `v_target` = Target object speed (m/s)

#### 6.1.2 Safe Stopping Distance

```
d_safe = v × t_reaction + (v² / (2 × μ × g))
```

Where:
- `t_reaction` = Reaction time (1.5s typical)
- `μ` = Friction coefficient
  - Dry asphalt: 0.8-0.9
  - Wet asphalt: 0.5-0.7
  - Snow: 0.2-0.3
  - Ice: 0.1-0.15
- `g` = Gravitational acceleration (9.81 m/s²)

#### 6.1.3 TTC Thresholds

| TTC Range | Action | Warning Level |
|-----------|--------|---------------|
| > 4.0s | Monitor only | None |
| 2.5-4.0s | Pre-charge brakes | Visual |
| 1.5-2.5s | Alert driver | Visual + Audible |
| 0.8-1.5s | Partial braking | Haptic + Audible |
| < 0.8s | Emergency braking | Full AEB activation |

### 6.2 Automatic Emergency Braking (AEB)

#### 6.2.1 Activation Logic

```python
def should_activate_aeb(ttc, certainty, velocity):
    if certainty < 0.9:
        return False  # Not confident enough

    if velocity < 5:  # m/s (~18 km/h)
        return False  # Too slow, driver can handle

    if ttc < 0.8:
        return True  # Critical situation

    if ttc < 1.5 and driver_not_reacting():
        return True  # Driver inattentive

    return False
```

#### 6.2.2 Braking Profile

**Jerk-limited braking:**
```
a(t) = a_max × (1 - e^(-t/τ))
```

Where:
- `a_max` = Maximum deceleration (8-10 m/s²)
- `τ` = Time constant (0.3-0.5s)
- Jerk limit: ±10 m/s³

**Distance to stop:**
```
d_stop = v²/(2 × a_max) + v × τ
```

### 6.3 Forward Collision Warning (FCW)

#### 6.3.1 Warning Strategy

**Multi-stage warning:**
```
Stage 1 (TTC < 3.0s): Visual indicator
Stage 2 (TTC < 2.0s): Visual + Audible beep
Stage 3 (TTC < 1.5s): Visual + Continuous tone + Brake prefill
Stage 4 (TTC < 0.8s): AEB activation
```

#### 6.3.2 False Positive Mitigation

```
Confidence Score = w₁ × sensor_quality +
                   w₂ × tracking_age +
                   w₃ × classification_confidence +
                   w₄ × trajectory_consistency

Activate only if: Confidence > 0.85
```

### 6.4 Pedestrian Detection and Protection

#### 6.4.1 Pedestrian Detection

**Multi-sensor approach:**
- Camera: HOG + CNN for pedestrian classification
- LiDAR: 3D point cloud clustering (height ~1.5-1.9m)
- Radar: Micro-Doppler signature (walking pattern)

**Detection challenges:**
- Occlusion (parked cars, street furniture)
- Varied appearance (clothing, posture)
- Unpredictable behavior
- Night-time detection

#### 6.4.2 Pedestrian Collision Mitigation

```
Risk Assessment:
1. Detect pedestrian
2. Predict trajectory (Kalman filter)
3. Calculate TTC
4. Assess collision probability

Actions:
- TTC > 2.5s: Monitor
- TTC 1.5-2.5s: Pre-brake, alert driver
- TTC < 1.5s: Full AEB + swerve if safe
```

**Swerve decision:**
```
if collision_imminent and can_swerve_safely():
    steering_angle = calculate_evasive_path()
    apply_steering_and_braking()
```

---

## 7. Adaptive Cruise Control

### 7.1 ACC Fundamentals

#### 7.1.1 Control Objectives

```
1. Maintain set speed (when road is clear)
   v_target = v_set

2. Maintain safe following distance (when following)
   d_target = v × t_gap + d_min

   Where:
   - t_gap = Time gap (1.0-2.5s, user-selectable)
   - d_min = Minimum distance (2-5m)
```

#### 7.1.2 Target Selection

**Multi-target scenario:**
```
for each detected_object:
    if is_in_ego_lane(object) and object.x > 0:
        potential_targets.add(object)

target = min(potential_targets, key=lambda obj: obj.x)
```

### 7.2 Longitudinal Control

#### 7.2.1 Speed Control (Cruise Mode)

**PID Controller:**
```
a_cmd = Kₚ × (v_set - v_ego) +
        Kᵢ × ∫(v_set - v_ego) dt +
        Kd × d/dt(v_set - v_ego)
```

Typical gains:
- `Kₚ = 0.3-0.5`
- `Kᵢ = 0.05-0.1`
- `Kd = 0.1-0.2`

#### 7.2.2 Following Control

**Gap control:**
```
e_gap = d_actual - d_target
a_cmd = Kₚ_gap × e_gap + Kd_gap × (v_target - v_ego)
```

Where:
```
d_target = v_ego × t_gap + d_min
v_target = v_leading_vehicle
```

#### 7.2.3 MPC-based ACC

**Optimization problem:**
```
Minimize: J = Σᵢ [q₁(vᵢ - v_ref)² + q₂(dᵢ - d_ref)² + q₃aᵢ² + q₄Δaᵢ²]

Subject to:
  - v_min ≤ vᵢ ≤ v_max
  - a_min ≤ aᵢ ≤ a_max
  - |Δaᵢ| ≤ jerk_max
  - dᵢ ≥ d_safe
```

### 7.3 ACC Operating Modes

| Mode | Condition | Behavior |
|------|-----------|----------|
| **Cruise** | No target ahead | Maintain set speed |
| **Following** | Target in range | Match target speed, maintain gap |
| **Approaching** | Closing on slower target | Decelerate smoothly |
| **Cut-in** | Vehicle enters lane | Quick deceleration |
| **Target Lost** | Target leaves lane | Resume to set speed |
| **Standstill** | Traffic stop (Stop&Go) | Full stop, resume on signal |

### 7.4 Stop & Go Functionality

#### 7.4.1 Standstill Management

```
if v_ego < 3 km/h and v_target == 0:
    apply_standstill_brake()
    state = STANDSTILL

if state == STANDSTILL:
    if v_target > 0:
        if standstill_time < 3s:
            resume_automatically()
        else:
            request_driver_confirmation()
```

#### 7.4.2 Comfort Braking

**Jerk minimization:**
```
a(t) = a_initial × (1 - t/T)³ + a_final × (t/T)³
```

This ensures:
- Smooth acceleration transitions
- Passenger comfort
- Reduced wear on brake system

### 7.5 ACC Safety Features

#### 7.5.1 Override Conditions

Driver override:
- Accelerator pedal > 10% → Disable distance control, allow speedup
- Brake pedal > 5% → Full system deactivation
- Steering > 90° → System deactivation (emergency maneuver)

#### 7.5.2 Fault Handling

```
if sensor_degraded():
    if degradation_level == MINOR:
        reduce_max_speed(to=80 km/h)
        reduce_time_gap(to=2.5s)
    elif degradation_level == MAJOR:
        alert_driver("ACC Limited Functionality")
        limit_speed(to=60 km/h)
    elif degradation_level == CRITICAL:
        deactivate_acc()
        alert_driver("ACC Unavailable")
```

---

## 8. SAE Automation Levels

### 8.1 Level Definitions

#### 8.1.1 SAE Level 0: No Automation

```
Driver performs all tasks:
- Steering
- Acceleration/deceleration
- Monitoring environment
- Fallback performance

Allowed features:
- Warnings (FCW, LDW)
- Momentary intervention (AEB in emergency)
```

#### 8.1.2 SAE Level 1: Driver Assistance

```
System controls:
- Steering OR
- Acceleration/deceleration

Driver performs:
- All other tasks
- Continuous monitoring
- Immediate takeover capability

Examples:
- Adaptive Cruise Control (ACC) alone
- Lane Keeping Assist (LKA) alone
```

#### 8.1.3 SAE Level 2: Partial Automation

```
System controls:
- Steering AND
- Acceleration/deceleration
(Simultaneously in specific conditions)

Driver performs:
- Continuous monitoring (hands-on, eyes-on)
- Immediate takeover
- Fallback performance

Examples:
- Tesla Autopilot
- GM Super Cruise
- Mercedes Drive Pilot (in traffic jams)

Requirements:
- Driver monitoring system
- Clear ODD (Operational Design Domain)
- Takeover request system
```

#### 8.1.4 SAE Level 3: Conditional Automation

```
System performs:
- All driving tasks within ODD
- Environment monitoring
- Fallback (with warning)

Driver performs:
- Respond to takeover request
- Fallback outside ODD

ODD Examples:
- Highway driving < 60 km/h
- Clear weather, marked lanes
- Specific geographic areas

Examples:
- Honda Legend (Traffic Jam Pilot)
- Mercedes Drive Pilot (Level 3 certified)

Minimum Risk Condition:
- If driver doesn't respond: slow to stop, hazard lights
```

#### 8.1.5 SAE Level 4: High Automation

```
System performs:
- All driving tasks within ODD
- Complete fallback within ODD

Driver:
- Not required within ODD
- May take over if desired

ODD Examples:
- Geo-fenced areas
- Specific weather conditions
- Designated roads/routes

Examples:
- Waymo robotaxis (Phoenix, SF)
- Cruise robotaxis
- Nuro delivery vehicles

Fallback:
- System brings vehicle to Minimum Risk Condition
- May continue in degraded mode
```

#### 8.1.6 SAE Level 5: Full Automation

```
System performs:
- All driving tasks
- All locations, all conditions
- Complete fallback everywhere

No ODD restrictions

Driver:
- Optional (passenger)
- No driving controls required

Status: Not yet achieved (as of 2025)

Challenges:
- Edge cases (construction, accidents)
- Extreme weather
- Unmapped areas
- Regulatory approval
```

### 8.2 Implementation Requirements by Level

| Requirement | L0 | L1 | L2 | L3 | L4 | L5 |
|-------------|----|----|----|----|----|----|
| Sensor redundancy | No | No | Yes | Yes | Yes | Yes |
| Driver monitoring | No | No | Yes | Yes | No | No |
| V2X communication | No | No | Optional | Recommended | Required | Required |
| HD Maps | No | No | Optional | Required | Required | Required |
| Remote assistance | No | No | No | Optional | Required | Required |
| Cybersecurity | Basic | Basic | Enhanced | Critical | Critical | Critical |
| OTA updates | No | Optional | Required | Required | Required | Required |
| Data recording (black box) | No | Optional | Required | Required | Required | Required |

---

## 9. Data Formats and Protocols

### 9.1 Sensor Data Formats

#### 9.1.1 LiDAR Point Cloud (PCD)

```protobuf
message PointCloud {
  message Point {
    float x = 1;              // meters
    float y = 2;              // meters
    float z = 3;              // meters
    uint32 intensity = 4;     // 0-255
    uint64 timestamp = 5;     // microseconds
    uint32 ring = 6;          // laser ring ID
  }

  repeated Point points = 1;
  uint64 timestamp = 2;
  string frame_id = 3;
}
```

#### 9.1.2 Radar Detections

```protobuf
message RadarDetection {
  message Target {
    float range = 1;          // meters
    float azimuth = 2;        // radians
    float elevation = 3;      // radians
    float range_rate = 4;     // m/s (radial velocity)
    float rcs = 5;            // dBsm (Radar Cross Section)
    float snr = 6;            // dB (Signal-to-Noise Ratio)
  }

  repeated Target targets = 1;
  uint64 timestamp = 2;
  string sensor_id = 3;
}
```

#### 9.1.3 Camera Image

```protobuf
message CameraImage {
  uint32 width = 1;
  uint32 height = 2;
  string encoding = 3;        // "rgb8", "bgr8", "mono8", etc.
  bytes data = 4;             // Raw image data
  uint64 timestamp = 5;
  string camera_id = 6;
  CameraInfo camera_info = 7;
}

message CameraInfo {
  repeated float K = 1;       // 3x3 intrinsic matrix
  repeated float D = 2;       // Distortion coefficients
  repeated float R = 3;       // 3x3 rectification matrix
  repeated float P = 4;       // 3x4 projection matrix
}
```

### 9.2 Perception Output Formats

#### 9.2.1 Detected Objects

```json
{
  "timestamp": 1735200000000000,
  "frame_id": "base_link",
  "objects": [
    {
      "id": 12345,
      "class": "car",
      "class_confidence": 0.97,
      "position": {"x": 25.3, "y": -2.1, "z": 0.0},
      "velocity": {"x": 22.5, "y": 0.2, "z": 0.0},
      "acceleration": {"x": -0.5, "y": 0.0, "z": 0.0},
      "dimensions": {"length": 4.5, "width": 1.8, "height": 1.5},
      "heading": 0.05,
      "covariance": [
        [0.1, 0, 0, 0, 0, 0],
        [0, 0.1, 0, 0, 0, 0],
        [0, 0, 0.05, 0, 0, 0],
        [0, 0, 0, 0.05, 0, 0],
        [0, 0, 0, 0, 0.01, 0],
        [0, 0, 0, 0, 0, 0.01]
      ],
      "tracking_age": 2.5,
      "prediction": {
        "horizon": 5.0,
        "trajectory": [
          {"t": 0.0, "x": 25.3, "y": -2.1},
          {"t": 1.0, "x": 47.8, "y": -2.0},
          {"t": 2.0, "x": 70.3, "y": -1.9},
          {"t": 3.0, "x": 92.8, "y": -1.8},
          {"t": 4.0, "x": 115.3, "y": -1.7},
          {"t": 5.0, "x": 137.8, "y": -1.6}
        ]
      }
    }
  ]
}
```

#### 9.2.2 Lane Information

```json
{
  "timestamp": 1735200000000000,
  "lanes": {
    "ego_left": {
      "coefficients": [0.1, 0.002, -0.00001, 0.0],
      "type": "dashed",
      "color": "white",
      "confidence": 0.95,
      "valid_range": {"y_min": 0, "y_max": 80}
    },
    "ego_right": {
      "coefficients": [-3.5, 0.001, -0.00001, 0.0],
      "type": "solid",
      "color": "white",
      "confidence": 0.93,
      "valid_range": {"y_min": 0, "y_max": 80}
    }
  },
  "ego_position": {
    "lateral_offset": -0.15,
    "heading_angle": 0.02
  }
}
```

### 9.3 Communication Protocols

#### 9.3.1 CAN Bus Messages

**ACC Status (ID: 0x300)**
```
Byte 0: ACC State (0=Off, 1=Standby, 2=Active, 3=Override, 4=Fault)
Byte 1: Set Speed (km/h)
Byte 2: Time Gap Setting (0.1s resolution)
Byte 3: Target Distance High Byte
Byte 4: Target Distance Low Byte (0.1m resolution)
Byte 5: Target Speed (km/h)
Byte 6-7: Reserved
```

**AEB Status (ID: 0x301)**
```
Byte 0: AEB State (0=Idle, 1=Armed, 2=Pre-braking, 3=Full-braking)
Byte 1-2: TTC (0.01s resolution)
Byte 3: Brake Pressure (0-100%)
Byte 4: Collision Risk (0=None, 1=Low, 2=Medium, 3=High, 4=Critical)
Byte 5-7: Reserved
```

#### 9.3.2 V2X (Vehicle-to-Everything)

**BSM (Basic Safety Message) - 802.11p**
```protobuf
message BasicSafetyMessage {
  uint32 msg_count = 1;       // Message sequence number
  bytes temporary_id = 2;     // Random vehicle ID (privacy)

  int32 lat = 3;              // Latitude (0.1 microdegree)
  int32 lon = 4;              // Longitude (0.1 microdegree)
  uint32 elevation = 5;       // Elevation (0.1m)

  uint32 speed = 6;           // Speed (0.02 m/s)
  uint32 heading = 7;         // Heading (0.0125 degrees)

  int32 accel_long = 8;       // Longitudinal accel (0.01 m/s²)
  int32 accel_lat = 9;        // Lateral accel (0.01 m/s²)
  int32 accel_vert = 10;      // Vertical accel (0.02 m/s²)

  VehicleSize size = 11;
  uint32 timestamp = 12;      // Milliseconds in minute
}
```

---

## 10. Safety Protocols and Validation

### 10.1 Functional Safety (ISO 26262)

#### 10.1.1 ASIL Levels

| Component | ASIL Level | Justification |
|-----------|------------|---------------|
| AEB | ASIL D | Critical safety function |
| Lane Keeping | ASIL B-D | Depends on implementation |
| ACC | ASIL B | Driver can override |
| Parking Assist | ASIL A | Low speed operation |
| Lane Departure Warning | ASIL A | Warning only |

#### 10.1.2 Safety Requirements

**For ASIL D (AEB):**
```
1. Sensor Redundancy: ≥2 independent sensors
2. Processing Redundancy: Dual ECUs with voting
3. Fault Detection: <100ms detection time
4. Fault Reaction: <200ms safe state transition
5. Diagnostic Coverage: >99%
6. Random Hardware Failure Rate: <10 FIT
7. Systematic Failure: Prevented through process
```

### 10.2 SOTIF (Safety of the Intended Functionality)

#### 10.2.1 Known Unsafe Scenarios

Examples requiring mitigation:
```
1. Overhanging cargo
   - LiDAR may not detect
   - Mitigation: Camera verification

2. Stationary vehicle on highway
   - Radar may filter as clutter
   - Mitigation: Camera + LiDAR fusion

3. Metallic debris on road
   - High radar RCS, low risk
   - Mitigation: Size estimation, classification

4. Black vehicles in tunnels
   - Low camera contrast
   - Mitigation: LiDAR primary, enhanced exposure

5. Pedestrians behind translucent barriers
   - Partial occlusion
   - Mitigation: Conservative detection, motion cues
```

#### 10.2.2 Performance Limits

```
Sensor Performance Envelope:
- LiDAR: Limited in heavy rain/fog
- Radar: Limited azimuth resolution
- Camera: Limited in low light, glare

System Performance Envelope:
- Max speed: 130 km/h (ACC), 80 km/h (LKA)
- Min visibility: 50m
- Road types: Paved, marked lanes
- Weather: Exclude heavy rain, snow, fog
```

### 10.3 Testing and Validation

#### 10.3.1 Test Scenarios (Euro NCAP)

**AEB Car-to-Car Tests:**
```
1. CCRs (Car-to-Car Rear Stationary)
   - Target: Stationary vehicle
   - Speed: 10, 20, 30, 40, 50 km/h

2. CCRm (Car-to-Car Rear Moving)
   - Target: Moving vehicle (20 km/h)
   - Speed: 30, 40, 50 km/h

3. CCRb (Car-to-Car Rear Braking)
   - Target: Decelerating vehicle (-6 m/s²)
   - Speed: 50 km/h
```

**AEB Pedestrian Tests:**
```
1. CPFA (Car-to-Pedestrian Farside Adult)
   - Adult crossing from right
   - Speed: 20, 30, 40, 50, 60 km/h

2. CPNC (Car-to-Pedestrian Nearside Child)
   - Child crossing from behind parked car
   - Speed: 20, 30, 40, 50 km/h

3. CPLA (Car-to-Pedestrian Longitudinal Adult)
   - Adult walking along road
   - Speed: 30, 40, 50, 60 km/h
```

#### 10.3.2 Simulation Testing

**Software-in-the-Loop (SIL):**
```
- Environment: Virtual scenarios
- Sensors: Simulated raw data
- Perception: Real algorithms
- Control: Real algorithms
- Vehicle: Simulated dynamics

Coverage: 10⁶ - 10⁹ km equivalent
```

**Hardware-in-the-Loop (HIL):**
```
- Environment: Simulated + real sensors
- ECUs: Real hardware
- Vehicle: Simulated dynamics
- CAN: Real bus

Coverage: 10⁵ - 10⁶ km equivalent
```

**Test Track:**
```
- Environment: Controlled real-world
- Sensors: Real
- Vehicle: Real with safety driver
- Targets: Soft dummies (EURO NCAP EVT, GVT)

Coverage: 10³ - 10⁴ km
```

**Public Road:**
```
- Environment: Real-world
- All systems real
- Safety driver + data logging

Coverage: 10⁶ - 10⁷ km required for Level 3+
```

### 10.4 Cybersecurity (ISO/SAE 21434)

#### 10.4.1 Threat Analysis

**Attack Vectors:**
```
1. Sensor Spoofing
   - GPS jamming/spoofing
   - Radar/LiDAR interference
   - Camera image injection

2. Communication Hijacking
   - CAN bus injection
   - V2X message forgery
   - OTA update tampering

3. Software Exploitation
   - Buffer overflow
   - Code injection
   - Privilege escalation
```

#### 10.4.2 Security Measures

```
1. Sensor Authentication
   - Encrypted communication
   - Digital signatures
   - Plausibility checks

2. Secure Boot
   - Verified boot chain
   - Code signing
   - Anti-rollback

3. Runtime Protection
   - Memory protection (MPU)
   - Intrusion detection
   - Anomaly monitoring

4. Communication Security
   - CAN authentication (CAN-FD + SecOC)
   - TLS for external communication
   - V2X message signing (PKI)

5. Update Security
   - Signed updates
   - Secure download (HTTPS)
   - Verification before installation
```

---

## 11. References

### 11.1 Standards and Regulations

1. **ISO 26262** - Road vehicles - Functional safety
2. **ISO/PAS 21448 (SOTIF)** - Safety Of The Intended Functionality
3. **ISO/SAE 21434** - Road vehicles - Cybersecurity engineering
4. **SAE J3016** - Taxonomy and Definitions for Terms Related to Driving Automation Systems
5. **Euro NCAP** - Assessment Protocol - Safety Assist
6. **UNECE R157** - Automated Lane Keeping Systems (ALKS)
7. **IEEE 802.11p** - Wireless Access in Vehicular Environments (WAVE)

### 11.2 Technical Papers

1. Kalman, R.E. (1960). "A New Approach to Linear Filtering and Prediction Problems"
2. 선행 연구. "Stanley: The Robot that Won the DARPA Grand Challenge"
3. 선행 연구. "YOLO: Real-Time Object Detection"
4. 선행 연구. "CARLA: An Open Urban Driving Simulator"
5. 선행 연구. "nuScenes: A multimodal dataset for autonomous driving"

### 11.3 Industry Resources

1. **Automotive Ethernet**: IEEE 802.3, 802.1 standards
2. **AUTOSAR**: Automotive Open System Architecture
3. **ROS 2**: Robot Operating System (for prototyping)
4. **OpenDRIVE**: Road network description format
5. **ASAM OpenX**: Simulation and testing standards

### 11.4 WIA Standards

- **WIA-INTENT**: Intent-based vehicle control interfaces
- **WIA-OMNI-API**: Universal automotive API gateway
- **WIA-SOCIAL**: V2V/V2X communication protocols
- **WIA-CLOUD**: Cloud services for connected vehicles
- **WIA-AI**: AI model training and deployment standards

---

## Appendix A: Example Calculations

### A.1 Safe Following Distance at 100 km/h

```
Given:
- Speed: v = 100 km/h = 27.78 m/s
- Reaction time: t = 1.5 s
- Friction (dry): μ = 0.8
- Gravity: g = 9.81 m/s²

Calculation:
- Reaction distance: d₁ = v × t = 27.78 × 1.5 = 41.67 m
- Braking distance: d₂ = v² / (2μg) = 27.78² / (2 × 0.8 × 9.81) = 49.17 m
- Total safe distance: d = d₁ + d₂ = 90.84 m

With 2-second time gap:
- d = 27.78 × 2 = 55.56 m < 90.84 m (unsafe for emergency stop)
- Recommended gap: 3.5 seconds minimum
```

### A.2 TTC Calculation Example

```
Given:
- Ego vehicle speed: v_ego = 90 km/h = 25 m/s
- Target vehicle speed: v_target = 50 km/h = 13.89 m/s
- Current distance: d = 50 m
- Safe distance: d_safe = 5 m

Relative velocity:
- v_rel = v_ego - v_target = 25 - 13.89 = 11.11 m/s

TTC:
- TTC = (d - d_safe) / v_rel
- TTC = (50 - 5) / 11.11 = 4.05 seconds

Action: Pre-charge brakes (TTC < 4.5s), visual warning
```

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-002 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
