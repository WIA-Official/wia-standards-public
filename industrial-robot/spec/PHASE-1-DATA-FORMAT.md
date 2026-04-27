# WIA-industrial-robot PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-industrial-robot
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-industrial-robot. The standard covers data exchange among
the entities that operate articulated, SCARA, parallel, gantry,
and collaborative industrial robots: robot vendors, system
integrators, operating organisations (factory, warehouse,
laboratory), safety-engineering teams, and the regulators and
certifying bodies that inspect robotic deployments. The format
captures robot identity and kinematic configuration, work-cell
layout, programmed task definition, motion telemetry, safety
configuration (functional-safety zones, speed and separation
monitoring), safety incidents, maintenance, and the
cybersecurity posture appropriate to ISO/IEC TR 22100-5 and
ISO 10218 expectations.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 17025:2017 (testing and calibration laboratories — for
  joint repeatability, accuracy, and TCP calibration)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- ISO 9283:1998 (manipulating industrial robots — performance
  criteria and related test methods)
- ISO 9787:2013 (manipulating industrial robots — coordinate
  systems and motion nomenclatures)
- ISO 10218-1:2011 / ISO 10218-2:2011 (robots and robotic devices
  — safety requirements; cited normatively)
- ISO/TS 15066:2016 (robots and robotic devices — collaborative
  robots; cited normatively for collaborative-operation
  requirements including biomechanical limits)
- ISO 8373:2021 (robotics — vocabulary)
- ISO/IEC TR 22100-5 (cybersecurity considerations for industrial
  machinery; cited normatively)
- ISO 13482:2014 (robots and robotic devices — safety
  requirements for personal-care robots; cited normatively when
  the deployment crosses into personal-care contexts)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IEC 61131-3 (PLC programming languages)
- IEC 61784-3 (functional-safety field-bus profiles)
- IEC 62061 (functional safety of safety-related electrical /
  electronic / programmable electronic control systems)
- OPC UA Robotics Companion Specification (cited as the
  preferred OPC UA envelope for industrial-robot data)

---

## §1 Scope

This PHASE document defines persistent shapes for the records that
flow across the operating life of an industrial-robot deployment:
from kinematic identification at commissioning, through programmed
task execution and motion telemetry, to incident response,
maintenance, and decommissioning. Implementations covered include:

- Robot controllers running vendor firmware.
- Work-cell PLCs that orchestrate robot, conveyor, and
  peripheral equipment.
- Collaborative-robot deployments where humans and robots share
  workspace under ISO/TS 15066 expectations.
- Robotic-cell programmers and integrators.
- Safety-engineering teams that perform task-based risk
  assessments and configure safety functions.
- Maintenance and reliability engineers.
- Regulators and certifying bodies that audit deployments under
  ISO 10218 / ISO/TS 15066.

Surgical robots, autonomous mobile robots (AMRs) operating in
unstructured environments, and exoskeletons fall under adjacent
WIA standards.

## §2 Robot Identity Record

Each industrial robot carries a stable identifier and the
kinematic, dynamic, and reach descriptors needed by integrators
and safety engineers.

```
robot:
  robotId         : string (uuidv7)
  siteRef         : string (operating site identifier; references
                       the deployment site)
  vendorRef       : string (institutional identifier of the
                       robot vendor)
  modelName       : string
  serialNumber    : string (manufacturer-assigned)
  controllerFirmwareVersion : string (Semantic Versioning 2.0.0)
  iso8373Class    : enum ("articulated" | "SCARA" |
                       "parallel-delta" | "cartesian-gantry" |
                       "cylindrical" | "spherical" |
                       "collaborative-articulated" |
                       "collaborative-SCARA")
  payloadKg       : number (rated payload at full reach)
  reachMillimetres : integer (maximum reach to wrist)
  axesCount       : integer (degrees of freedom; typically 6 for
                       articulated, 4 for SCARA)
  repeatabilityMm : number (per ISO 9283 RP test conditions)
  protectionRating : string (IP code per the relevant IEC
                       protection-class standard)
```

The kinematic descriptor is published by the vendor as a
content-addressed companion document referenced from the robot
record; integrators consume the kinematic descriptor when
programming the robot's offline path planning.

## §3 Work-Cell Layout

```
workCell:
  cellId          : string (uuidv7)
  siteRef         : string
  robotRefs       : array of string (robotId list)
  peripheralAssetRefs : array of string (conveyors, vision
                       systems, end-effectors, tool changers)
  cellLayoutRef   : string (content-addressed URI of the layout
                       drawing — typically a STEP / SAT / IGES
                       file with workspace boundaries)
  iso10218SafetyClass : enum ("supervised-stop" |
                       "hand-guiding" |
                       "speed-and-separation-monitoring" |
                       "power-and-force-limiting")
  collaborativeOperation : boolean (true when the cell operates
                       under ISO/TS 15066 collaborative modes)
```

`iso10218SafetyClass` selects the collaborative mode the cell is
allowed to use; transitions between modes follow the safety-
engineering matrix defined in PHASE-3.

## §4 Programmed Task Record

```
task:
  taskId          : string (uuidv7)
  cellId          : string (uuidv7)
  taskKind        : enum ("pick-and-place" | "welding" |
                       "painting" | "assembly" | "machine-
                       tending" | "palletising" | "deburring" |
                       "vision-guided-pick" | "force-controlled-
                       insertion" | "human-collaboration" |
                       "user-defined")
  programArtefactRef : string (content-addressed URI of the
                       programmed task file in the vendor's
                       native format)
  cycleTimeBudgetSec : number (planned cycle-time budget per
                       task instance)
  toolingRef      : string (end-effector identifier)
```

Programmed tasks carry their content-address so that retrospective
review can reproduce the program that was running at any given
time.

## §5 Motion Telemetry Record

Motion telemetry samples the joint-space and Cartesian-space
state at the operator's chosen cadence. The cadence is bounded
above by the controller's bus rate and below by the operator's
forensic-history requirement.

```
motionSample:
  sampleId        : string (uuidv7)
  robotId         : string (uuidv7)
  capturedAt      : string (ISO 8601 / RFC 3339, sub-millisecond
                       precision)
  jointPositionsRad : array of number (per-axis position in
                       radians, ordered per ISO 9787 base-to-
                       wrist convention)
  jointVelocitiesRadPerS : array of number
  jointTorquesNm  : array of number
  tcpPositionMm   : array of number (X, Y, Z in millimetres,
                       cell-frame)
  tcpOrientationQuat : array of number (quaternion W X Y Z)
  tcpForceN       : array of number (Fx, Fy, Fz when force-torque
                       sensing is fitted)
  tcpTorqueNm     : array of number (Tx, Ty, Tz)
  taskRef         : string (taskId; absent during teach-mode)
  programLine     : integer (current line in the programmed task)
```

Programmes that publish externally cited motion-quality KPIs
include the underlying motion samples in the evidence package so
that downstream consumers can recompute the KPIs.

## §6 Safety Configuration Record

```
safetyConfiguration:
  configId        : string (uuidv7)
  cellId          : string (uuidv7)
  safetyZones     : array of SafetyZone
  speedLimits     : array of SpeedLimit
  separationMonitoring : object (per ISO/TS 15066 §5.5.4
                       speed-and-separation thresholds)
  powerForceLimits : object (per ISO/TS 15066 Annex A
                       biomechanical limits)
  fieldBusProfile : enum ("PROFISafe" | "FSoE" | "CIPSafety" |
                       "SafetyOverEthernet-equivalent" |
                       "user-defined")
  iec62061SilLevel : enum ("SIL-1" | "SIL-2" | "SIL-3")
  iso13849PerformanceLevel : enum ("PL-a" | "PL-b" | "PL-c" |
                       "PL-d" | "PL-e")
```

Safety configurations are revised through controlled change
(PHASE-3 §5); prior configurations remain addressable.

## §7 Safety Incident Record

Collisions, e-stops, light-curtain breaches, and collaborative-
mode biomechanical-limit exceedances are recorded as safety
incidents. Records carry the reconstruction window of motion
samples that surround the incident so that retrospective analyses
can recover the joint trajectory.

```
safetyIncident:
  incidentId      : string (uuidv7)
  cellId          : string (uuidv7)
  occurredAt      : string (ISO 8601 / RFC 3339)
  category        : enum ("e-stop-press" | "light-curtain-breach"
                       | "speed-limit-exceedance" |
                       "force-limit-exceedance" |
                       "separation-violation" |
                       "operator-injury" |
                       "robot-self-collision" |
                       "external-collision" |
                       "tool-failure")
  severity        : enum ("near-miss" | "minor-injury" |
                       "major-injury" | "fatality")
  reconstructionWindowMs : integer (motion-sample window captured
                       for the incident, typically 1000-5000 ms
                       around `occurredAt`)
  reportedToAuthorityRef : string (regulator report reference;
                       opaque)
  rootCauseRef    : string (URI of the root-cause investigation)
```

Incidents of `severity` ≥ `major-injury` MUST be reported to the
relevant occupational-safety authority within the period the
authority requires.

## §8 Maintenance Record

```
maintenance:
  recordId        : string (uuidv7)
  robotId         : string (uuidv7)
  performedAt     : string (ISO 8601)
  category        : enum ("preventive-scheduled" |
                       "predictive-condition-based" |
                       "corrective-emergency" |
                       "calibration-tcp" |
                       "calibration-iso-9283" |
                       "joint-encoder-replacement" |
                       "harmonic-drive-replacement" |
                       "controller-firmware-update")
  technicianRef   : string (institutional identifier)
  workOrderRef    : string (CMMS work-order reference)
  partsConsumed   : array of PartConsumption
  durationMinutes : integer
```

## §9 Cybersecurity Posture Record

The work-cell's cybersecurity posture per ISO/IEC TR 22100-5 and
IEC 62443 is recorded so that downstream consumers can verify the
deployment's security level alongside its safety configuration.

```
cyberPosture:
  postureId       : string (uuidv7)
  cellId          : string (uuidv7)
  iec62443Zone    : string
  controlsInForce : array of string
  lastReviewedAt  : string (ISO 8601)
  reviewerRef     : string
```

## §10 End-Effector and Tool Record

End-effectors (grippers, welding torches, paint applicators,
force-torque-wrist sensors, screwdriver tools, vacuum cups) are
catalogued separately from the robot record because end-
effectors swap independently of the host robot.

```
endEffector:
  effectorId      : string (uuidv7)
  vendorRef       : string (institutional identifier)
  modelName       : string
  serialNumber    : string
  effectorClass   : enum ("two-finger-gripper" |
                       "three-finger-gripper" |
                       "vacuum-cup" | "magnetic" |
                       "welding-torch" | "paint-applicator" |
                       "force-torque-wrist" | "screwdriver" |
                       "deburring-tool" | "tool-changer" |
                       "user-defined")
  payloadKgAtTcp  : number (rated payload referenced to the TCP
                       offset)
  toolCenterPointMm : array of number (X, Y, Z offset from the
                       robot wrist mounting flange)
  toolCenterPointOrientationDeg : array of number (RX, RY, RZ
                       Euler angles relative to the wrist frame)
  payloadInertiaKgm2 : array of number (Ixx, Iyy, Izz, Ixy,
                       Ixz, Iyz)
```

End-effector swap events are recorded under maintenance (PHASE-1
§8) so that cycle-time and incident records can be correlated to
the active end-effector at the time.

## §11 Vision-System Binding (Optional)

Cells that include vision-guided tasks bind a vision-system
asset to the robot record so that the calibration between
camera frame and robot base frame is auditable.

```
visionBinding:
  bindingId       : string (uuidv7)
  robotId         : string (uuidv7)
  visionAssetId   : string (uuidv7)
  calibrationRef  : string (content-addressed URI of the hand-
                       eye / hand-camera calibration result)
  reprojectionErrorPx : number (RMS reprojection error per the
                       calibration report)
  validatedAt     : string (ISO 8601 / RFC 3339)
```

Vision-system recalibration events (after camera replacement,
camera-bracket impact, or scheduled cadence) emit new bindings;
prior bindings remain addressable for retrospective analysis of
vision-related incidents.

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every operating cell and honour the
content-addressing rules in §3-§9.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-industrial-robot
- **Last Updated:** 2026-04-27
