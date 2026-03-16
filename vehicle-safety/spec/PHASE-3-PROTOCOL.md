# WIA-AUTO-022 PHASE 3: Protocol

> **Version:** 1.0.0  
> **Status:** Active  
> **Last Updated:** 2025-12-27  
> **Focus:** ISO 26262, ASIL classification, testing protocols

---

## Overview

Phase 3 implements safety protocols and compliance frameworks governing system behavior. This phase integrates ISO 26262 functional safety, crash testing methodologies (FMVSS, Euro NCAP), and real-time communication protocols.

**弘益人間 (Benefit All Humanity)** - Rigorous protocols ensure safety systems perform reliably, protecting lives in critical moments.

---

## ISO 26262 Integration

### ASIL Classification

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

### Safety Requirements for ASIL-D (Airbag Systems)

1. **Dual Microcontroller Architecture**: Independent processors with cross-checking
2. **Watchdog Timers**: Independent clock sources, < 100ms timeout
3. **ECC RAM**: Error correction for all safety-critical data
4. **Diagnostic Coverage**: > 99% fault detection capability
5. **Single Point Fault Metric**: > 99%
6. **Latent Fault Metric**: > 90%

---

## FMVSS Compliance Testing

### FMVSS 208 - Occupant Protection

**Test Requirements:**
- 35 mph frontal rigid barrier crash (belted occupants)
- 25 mph frontal rigid barrier crash (unbelted occupants)
- Hybrid III 50th percentile male and 5th percentile female dummies

**Injury Criteria Thresholds:**
- HIC-15 < 700
- Chest acceleration (3ms) < 60g
- Femur load < 10 kN (each leg)

### FMVSS 214 - Side Impact Protection

**Test Requirements:**
- 33.5 mph moving deformable barrier (MDB)
- Side impact dummies (SID-IIs or WorldSID)

**Injury Criteria:**
- Thoracic Trauma Index (TTI) < 85
- Pelvic force < 6.0 kN
- Abdominal force < 2.5 kN

### FMVSS 216a - Roof Crush Resistance

**Test Requirements:**
- Quasi-static roof strength test
- Strength-to-Weight Ratio (SWR) ≥ 3.0
- Both driver and passenger sides tested

---

## Euro NCAP Protocol

### Rating Methodology (2025)

**Overall Rating Calculation:**
- Adult Occupant: 40% weight (≥ 80% for 5 stars)
- Child Occupant: 20% weight (≥ 80% for 5 stars)
- Vulnerable Road Users: 20% weight (≥ 70% for 5 stars)
- Safety Assist: 20% weight (≥ 70% for 5 stars)

### Test Scenarios

**Frontal Impact:**
- 40% offset deformable barrier at 40 mph
- Full-width rigid barrier at 31 mph

**Side Impact:**
- Moving deformable barrier at 50 km/h
- Pole impact at 32 km/h

**Pedestrian/Cyclist:**
- Pedestrian AEB tests at 20-60 km/h
- Cyclist AEB tests at crossing scenarios

**Active Safety:**
- AEB car-to-car tests
- Lane support systems evaluation
- Speed assistance systems assessment

---

## Real-Time Communication Protocols

### V2V Safety Messages

**Basic Safety Message (BSM):**
- Frequency: 10 Hz
- Latency: < 100ms
- Content: Position, velocity, heading, acceleration
- Range: 300 meters minimum

**Emergency Electronic Brake Light (EEBL):**
- Trigger: Hard braking event (> 0.4g deceleration)
- Priority: High
- Propagation: Multi-hop (up to 3 hops)

### Security

- Message authentication using ECDSA signatures
- Certificate management (short-lived pseudonymous certificates)
- Privacy protection (certificate rotation every 5 minutes)

---

## Cybersecurity Standards (ISO/SAE 21434)

### Threat Analysis

**Common Attack Vectors:**
1. Remote exploitation via connected services
2. Physical access through OBD-II port
3. Supply chain compromise
4. Wireless key fob relay attacks

### Security Controls

- Network segmentation (safety-critical on isolated CAN)
- Intrusion detection systems
- Secure boot (verified firmware signatures)
- Secure OTA updates (signed and encrypted)

---

## Testing Workflows

### Crash Test Execution

1. Test Setup (instrumentation, dummy positioning)
2. Pre-test Validation (system checks, calibration)
3. Test Execution (high-speed cameras, data acquisition)
4. Post-test Analysis (data extraction, injury calculation)
5. Automated Upload (results to certification portal)
6. Compliance Verification (automated checks against thresholds)
7. Certificate Generation (W3C Verifiable Credential)

### Automated Compliance Checking

```yaml
compliance_rules:
  - rule_id: NCAP-AO-001
    description: "Adult occupant frontal HIC-15 must be < 700"
    test: frontal_offset
    metric: driver.hic15
    threshold: 700
    operator: less_than
    
  - rule_id: FMVSS-208-001
    description: "Chest acceleration 3ms < 60g"
    test: frontal_rigid
    metric: driver.chest_acceleration_3ms
    threshold: 60
    operator: less_than
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA | MIT License
