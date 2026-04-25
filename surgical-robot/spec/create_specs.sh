#!/bin/bash

# PHASE-1: System Architecture
cat > PHASE-1-SYSTEM-ARCHITECTURE.md << 'EOF'
# WIA-MED-006 PHASE 1: System Architecture Specification

## Overview
This specification defines the fundamental architecture for surgical robotic systems compliant with WIA-MED-006 standard.

## 1. System Components

### 1.1 Surgeon Console
- **3D Vision System**: 1920x1080 per eye minimum, 60 Hz, stereo separation
- **Master Manipulators**: 7-DOF, 0.1mm position resolution, force feedback capable
- **Foot Pedals**: Minimum 4 pedals for camera, clutch, and energy activation
- **Ergonomic Seating**: Adjustable height, arm rests, head support

### 1.2 Patient-Side Cart
- **Robotic Arms**: 3-4 arms minimum, 7-DOF each
- **RCM Mechanism**: Remote Center of Motion with ±0.5mm deviation tolerance
- **Tool Mounting**: Standard interface for interchangeable instruments
- **Positioning**: Laser targeting or manual positioning with locking mechanism

### 1.3 Vision Cart
- **Image Processing**: Real-time processing, <20ms latency
- **Light Source**: LED 300W equivalent, adjustable intensity
- **Recording**: Optional 1080p60 recording capability

## 2. Communication Architecture

### 2.1 Control Bus
- **Protocol**: CAN bus or equivalent, 1 Mbps minimum
- **Update Rate**: 1000 Hz for position commands
- **Latency**: <1ms end-to-end
- **Redundancy**: Dual independent channels

### 2.2 Vision Data
- **Bandwidth**: 3 Gbps minimum for 3D HD video
- **Compression**: H.265 or better
- **Latency**: <5ms encoding/decoding

## 3. Safety Architecture

### 3.1 Hardware Safety
- Independent safety PLC
- Dual watchdog timers
- Emergency stop circuit (<50ms response)
- Mechanical limit stoppers

### 3.2 Software Safety
- IEC 62304 Class C compliance
- Fail-safe default behaviors
- Continuous self-diagnostics
- Event logging (7-year retention)

## 4. Interoperability

### 4.1 Standard Interfaces
- **Console-Cart**: WIA-MED-006-CTRL protocol (JSON/MessagePack)
- **Vision**: WIA-MED-006-VIS protocol (H.265/AV1)
- **Tools**: WIA-MED-006-TOOL protocol (sensor data, calibration)

### 4.2 Compatibility
- Components from different manufacturers must be interoperable
- Standard connector types and pinouts
- Plug-and-play capability with auto-detection

## 5. Performance Requirements

- **Position Accuracy**: ±0.1mm
- **Repeatability**: ±0.02mm
- **Control Loop**: ≥1000 Hz
- **System Latency**: ≤10ms
- **Uptime**: >98%

---

**Status**: ✅ Complete  
**Version**: 1.0.0  
**Date**: 2025-01-01

© 2025 WIA · MIT License
EOF

# PHASE-2: Haptic Feedback
cat > PHASE-2-HAPTIC-FEEDBACK.md << 'EOF'
# WIA-MED-006 PHASE 2: Haptic Feedback Specification

## Overview
This specification defines haptic feedback requirements for surgical robotic systems.

## 1. Force Sensing

### 1.1 Sensor Requirements
- **Force Range**: 0-10N continuous, 0-20N peak
- **Resolution**: 0.1N
- **Accuracy**: ±0.1N or 2% of reading
- **Sampling Rate**: ≥1000 Hz
- **Sensor Types**: Strain gauge, piezoelectric, or 6-axis force/torque

### 1.2 Mounting Locations
- Tool shaft (bending/torsion)
- Tool jaw (grip force)
- Tool base (6-axis F/T)

## 2. Haptic Rendering

### 2.1 Processing Pipeline
1. **Sensor Data Acquisition** (1000 Hz)
2. **Signal Processing**: Noise filtering (Kalman), drift correction
3. **Force Mapping**: Scaling, saturation (max 5N), deadzone (<0.1N)
4. **Tissue Modeling**: K=1-50 N/mm, damping B=0.1-5 Ns/mm
5. **Actuator Drive**: Motor current commands

### 2.2 Performance
- **Update Rate**: 1000 Hz minimum (stability requirement)
- **Total Latency**: <1ms (transparency requirement)
- **Stability**: Guaranteed for passive environments

## 3. Haptic Actuators

### 3.1 Actuator Specifications
- **Force Range**: 0-5N continuous
- **Bandwidth**: DC to 100 Hz minimum
- **Resolution**: 0.05N
- **Backdrivability**: Low friction/inertia

### 3.2 Actuator Types (any compliant)
- DC motors with gearing
- Voice coil actuators
- Piezoelectric actuators
- Pneumatic actuators

## 4. Tissue Models

### 4.1 Standard Tissue Parameters
```
Fat:      K = 1-10 kPa,   High viscoelasticity
Muscle:   K = 10-100 kPa, Medium viscoelasticity
Liver:    K = 5-20 kPa,   High viscoelasticity
Kidney:   K = 10-30 kPa,  Medium viscoelasticity
Vessel:   K = 100-1000 kPa, Low viscoelasticity
Tumor:    K = 50-500 kPa, Low viscoelasticity
Bone:     K = 10-20 GPa,  Very low viscoelasticity
```

### 4.2 Modeling Approaches
- Linear elastic: F = K × x
- Viscoelastic (Kelvin-Voigt): F = K × x + B × v
- Nonlinear: F = K₁×x + K₂×x² + K₃×x³
- Hunt-Crossley (contact): F = K × x^n × (1 + λ × v)

## 5. Validation

### 5.1 Benchtop Testing
- Force accuracy verification
- Frequency response measurement
- Stability testing

### 5.2 Tissue Phantom Testing
- Silicone phantoms with calibrated stiffness
- Porcine tissue comparison
- User studies (N≥10 surgeons)

## 6. Optional Advanced Features

- Texture rendering (vibrotactile 10-300 Hz)
- Thermal feedback
- Multi-point tactile arrays
- Adaptive rendering based on user preference

---

**Status**: ✅ Complete  
**Version**: 1.0.0  
**Date**: 2025-01-01

© 2025 WIA · MIT License
EOF

# PHASE-3: Precision Control
cat > PHASE-3-PRECISION-CONTROL.md << 'EOF'
# WIA-MED-006 PHASE 3: Precision Control Specification

## Overview
This specification defines precision control requirements including motion scaling, tremor filtering, and navigation.

## 1. Motion Scaling

### 1.1 Scaling Ratios
- **1:1**: Fast repositioning, no scaling
- **3:1**: Standard surgery (default)
- **5:1**: High-precision tasks
- **10:1**: Microsurgery (optional)

### 1.2 Scaling Requirements
- **Transition**: Smooth blending, <100ms transition time
- **Linearity**: ±2% across full range
- **User Control**: Surgeon-selectable via foot pedal or voice
- **Adaptive**: Optional context-aware auto-adjustment

### 1.3 Implementation
```
X_slave = (1/S) × X_master
where S = scaling factor (1, 3, 5, or 10)

Adaptive scaling:
IF (near critical structure) THEN S = 5
ELSE IF (velocity > threshold) THEN S = 1  
ELSE S = 3
```

## 2. Tremor Filtering

### 2.1 Tremor Characteristics
- Physiological tremor: 8-12 Hz, 0.1-0.5mm amplitude
- Fatigue tremor: 4-8 Hz, 0.5-2.0mm amplitude
- Pathological tremor: 4-12 Hz, 1.0-5.0mm amplitude

### 2.2 Filtering Requirements
- **Tremor Reduction**: ≥90%
- **Signal Delay**: ≤10ms
- **Phase Distortion**: ≤5°
- **Passband**: DC to 3 Hz (preserve intended motion)

### 2.3 Multi-Stage Filter
1. **Low-pass**: Butterworth, 3 Hz cutoff
2. **Kalman Filter**: State estimation [position, velocity, acceleration]
3. **Adaptive**: Increase strength when tremor detected
4. **Predictive**: 100ms forward prediction for latency compensation

## 3. Position Control

### 3.1 Accuracy Requirements
- **Position Accuracy**: ±0.1mm
- **Repeatability**: ±0.02mm  
- **Path Accuracy**: ±0.5mm
- **Settling Time**: <100ms

### 3.2 Control Loop
- **Update Rate**: ≥1000 Hz
- **Controller**: PID or model-based
- **Feedforward**: Optional for improved tracking

## 4. Collision Avoidance

### 4.1 Real-time Collision Detection
- **Check Rate**: 100 Hz minimum
- **Geometric Model**: Bounding volumes (cylinders, capsules)
- **Safety Distance**: 10mm warning, 5mm stop
- **Response**: Immediate stop + haptic/visual warning

### 4.2 Prediction
- **Lookahead**: 500ms trajectory prediction
- **Alternative Paths**: Suggest collision-free routes
- **Recovery**: Safe retraction to collision-free position

## 5. Virtual Fixtures

### 5.1 Fixture Types
- **Forbidden Regions**: Repulsive force around critical structures
- **Guidance Regions**: Attractive force along planned path
- **Scaling Regions**: Variable scaling by workspace zone

### 5.2 Implementation
```
Forbidden Region:
IF tool enters forbidden zone THEN
  F_repulsive = K × penetration_depth
  Push tool away + haptic warning
END IF

Guidance:
F_guidance = K_guide × path_error
```

## 6. Image-Guided Navigation

### 6.1 Registration
- **Target Registration Error (TRE)**: <2mm
- **Fiducial Registration Error (FRE)**: <1mm
- **Methods**: Anatomical landmarks, surface matching, paired-point

### 6.2 Tracking
- **Update Rate**: 30 Hz minimum
- **Accuracy**: ±1mm throughout workspace
- **Deformation**: Real-time tissue tracking (optional)

### 6.3 AR Overlays
- Vascular maps (red: arteries, blue: veins)
- Tumor boundaries (yellow outlines)
- Safety zones (red forbidden regions)
- Planned trajectories (green paths)
- Distance measurements

---

**Status**: ✅ Complete  
**Version**: 1.0.0  
**Date**: 2025-01-01

© 2025 WIA · MIT License
EOF

# PHASE-4: Safety Protocols
cat > PHASE-4-SAFETY-PROTOCOLS.md << 'EOF'
# WIA-MED-006 PHASE 4: Safety Protocols Specification

## Overview
This specification defines comprehensive safety requirements for surgical robotic systems.

## 1. Safety Standards Compliance

### 1.1 Mandatory Standards
- **IEC 60601-1**: Medical electrical equipment - General safety
- **IEC 60601-2-77**: Robotically-assisted surgical equipment
- **ISO 13482**: Safety requirements for personal care robots
- **ISO 14971**: Medical device risk management
- **IEC 62304**: Medical device software lifecycle (Class C)

## 2. Hardware Safety

### 2.1 Emergency Stop System
- **E-Stop Buttons**: Minimum 2 (console + bedside)
- **Response Time**: <50ms to power cutoff
- **Action**: Immediate motor power disconnect + brake engagement
- **Indicator**: Red LED + audible alarm
- **Recovery**: Manual reset required after E-stop

### 2.2 Safety PLC
- **Independence**: Separate from main control system
- **Redundancy**: Dual independent channels
- **Watchdog**: <100ms timeout
- **Self-Test**: Power-on + periodic (every 1 hour)

### 2.3 Mechanical Limits
- **Hard Stops**: Physical mechanical limiters at joint extremes
- **Limit Switches**: Redundant switches before hard stops
- **Breakaway Links**: Designed failure points (500N threshold)

## 3. Software Safety

### 3.1 Fail-Safe Behaviors
- **Communication Loss**: Stop within 1 second, hold position
- **Sensor Failure**: Switch to redundant sensor or safe stop
- **Software Crash**: Watchdog reset, safe mode boot
- **Power Loss**: Battery backup (5-10 min), controlled shutdown

### 3.2 Safety-Critical Functions
- **Workspace Limits**: Software boundaries, warn at 90%, stop at 100%
- **Force Limits**: Configurable max force (default 15N), hard limit 20N
- **Velocity Limits**: Max 100mm/s near patient, 500mm/s repositioning
- **Collision Detection**: Real-time arm-arm and arm-patient checks

### 3.3 Software Verification
- **Static Analysis**: MISRA C compliance
- **Unit Testing**: >90% code coverage
- **Integration Testing**: All safety scenarios
- **Formal Verification**: Safety-critical algorithms

## 4. Procedural Safety

### 4.1 Pre-Surgery Checklist
- [ ] Patient identity confirmed
- [ ] Surgical site marked
- [ ] System self-test passed
- [ ] Tool calibration verified
- [ ] Emergency stop tested
- [ ] Backup plan established
- [ ] Team roles assigned

### 4.2 Intra-operative Monitoring
- **Control Latency**: Continuous monitoring, alert if >20ms
- **Position Error**: Alert if >1.0mm deviation
- **Motor Current**: Alert if >90% rated
- **Temperature**: Alert if >40°C
- **Network (remote)**: Alert if latency >100ms or packet loss >0.1%

### 4.3 Event Logging
- **Log All Events**: System events, user actions, safety events, performance
- **Format**: [Timestamp] [Severity] [Component] [Message]
- **Storage**: Local (1 hour buffer) + central (7 years)
- **Encryption**: AES-256 at rest
- **Audit Trail**: Tamper-proof, legally compliant

## 5. Emergency Procedures

### 5.1 Emergency Classification
- **Level 1**: Information (monitor)
- **Level 2**: Caution (slow down)
- **Level 3**: Warning (modify action)
- **Level 4**: Emergency (stop, handover)
- **Level 5**: Crisis (open conversion, CPR)

### 5.2 Local Handover Protocol
1. **Recognition** (1-5s): System failure detected
2. **Assessment** (5-15s): Severity evaluation
3. **Decision** (5-10s): Continue vs. handover
4. **Transition** (1-3min): Robot to safe position, detach, bedside surgeon takes over
5. **Documentation**: Incident logging

## 6. Quality Assurance

### 6.1 Maintenance Schedule
- **Daily**: Visual inspection, self-test, cleaning
- **Weekly**: Deep clean, supplies check, log review
- **Monthly**: Precision calibration, preventive maintenance
- **Quarterly**: Comprehensive inspection, external audit
- **Annual**: Full system validation, regulatory compliance check

### 6.2 Performance Metrics (KPIs)
- **Uptime**: >98%
- **Error Rate**: <0.1% per surgery
- **Mean Time to Repair**: <2 hours
- **Complication Rate**: <5%
- **Conversion Rate**: <2%

## 7. Training Requirements

### 7.1 Mandatory Training
- **System Operation**: 40 hours minimum
- **Safety Protocols**: 8 hours
- **Emergency Procedures**: 4 hours + quarterly drills
- **Simulator**: GEARS score ≥20

### 7.2 Certification
- **Initial**: WIA-MED-006 Level 2 minimum
- **Renewal**: Every 5 years
- **Continuing Education**: 10 CME credits annually

---

**Status**: ✅ Complete  
**Version**: 1.0.0  
**Date**: 2025-01-01

© 2025 WIA · MIT License
EOF

echo "All spec files created successfully!"
