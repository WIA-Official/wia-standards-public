# WIA-AUTO-002 PHASE 1 — Data Format Specification

**Standard:** WIA-AUTO-002
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

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


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.
