# WIA-ROB-020: Robot Operating System Standard Specification v1.0

Normative contracts for robot runtime configuration, component coordination, motion authority, joint control, telemetry, fault handling, and conformance assessment.

## 1. Introduction

### 1.1 Purpose

WIA-ROB-020 defines an interoperable supervisory and execution contract for a robot operating system. Its purpose is to make configuration, command ownership, actuator interaction, timing, state observation, and failure handling explicit and testable across implementations.

A conforming implementation combines a supervisory management interface with a local execution mechanism. Supervisory requests may authorize work, but local mechanisms enforce command validity, resource ownership, deadlines, and stop behavior at the actuator boundary.

This edition establishes the identifier **WIA-ROB-020**, version **1.0**, the English name **Robot Operating System**, and the Korean name **로봇 운영체제**. It inherits no WIA technical profile, field registry, thresholds, or certification rules. The technical values in this document are definitions of this specification.

The term “robot operating system” does not require a particular kernel, middleware, programming language, or software product. Conformance does not establish compatibility with software named ROS or ROS 2.

### 1.2 Scope

**In scope:**

- Runtime and component discovery.
- Configuration of controlled joints, components, and coordinate frames.
- Dependency validation and runtime lifecycle management.
- Exclusive, expiring motion authority.
- A complete joint-velocity command contract.
- Local actuator adapters, feedback acquisition, and execution fencing.
- Monotonic timing, watchdogs, and bounded command validity.
- Joint-state, frame-transform, and component-health telemetry.
- Fault reporting, stop initiation, and stop confirmation.
- Authentication, authorization, auditability, and privacy controls.
- REST management interfaces, bounded event replay, and conformance assessment.

**Out of scope:**

- Kernel implementation, processor architecture, and programming-language bindings.
- Navigation, task planning, perception algorithms, and inverse kinematics.
- Position-trajectory, force, impedance, and whole-body control interfaces.
- Robot mechanical design and application-specific risk assessment.
- Emergency-stop circuit design or functional-safety certification.
- Guarantees of collision avoidance or human-safe contact.
- Automatic transfer of active motion control between redundant runtimes.
- A universal actuator fieldbus or binary device protocol.

Additional capabilities MAY coexist with this specification. They MUST NOT bypass its authority, lifecycle, deadline, or stop requirements when controlling joints inside the conforming boundary.

### 1.3 Normative references

The following documents apply to the specific mechanisms referenced:

- **RFC 2119**, *Key words for use in RFCs to Indicate Requirement Levels*.
- **RFC 8174**, *Ambiguity of Uppercase vs Lowercase in RFC 2119 Key Words*.
- **RFC 8259**, *The JavaScript Object Notation (JSON) Data Interchange Format*.
- **RFC 8446**, *The Transport Layer Security (TLS) Protocol Version 1.3*.
- **RFC 9110**, *HTTP Semantics*.
- **JSON Schema, Draft 2020-12**, Core and Validation specifications.

These references do not establish robot functional-safety approval.

### 1.4 Terms and definitions

| Term | Definition |
|---|---|
| Runtime | Authority-bearing software instance coordinating a defined set of robot components and joints. |
| Robot identifier | Stable identifier of the robot installation represented by a runtime. |
| Runtime identifier | Stable identifier of a managed runtime endpoint. |
| Boot identifier | Identifier of one uninterrupted incarnation of the runtime’s authority and monotonic clock domain. |
| Component | Registered driver, controller, application, or monitor with declared dependencies. |
| Driver adapter | Local interface translating runtime operations into device-specific feedback and actuator interactions. |
| Controller | Component producing or applying ordinary actuator targets within the runtime contract. |
| Joint | Independently represented rotary or linear degree of freedom controlled by this specification. |
| Control period | Configured interval between scheduled local execution releases. |
| Motion lease | Exclusive, time-limited authorization for one authenticated principal to submit motion commands. |
| Fencing epoch | Increasing generation number invalidating commands from earlier authority generations. |
| Command acceptance | Atomic installation of a validated command as the current command; distinct from actuator execution. |
| Execution handoff | Delivery of validated targets to driver adapters during a local control cycle. |
| Current command | The sole nonterminal velocity command installed for a runtime. |
| Stop procedure | Commissioned local procedure that arrests motion while preserving application-required support or holding behavior. |
| Stop confirmation | Positive execution-side fencing acknowledgment and qualifying fresh velocity measurements. |
| Protective input | Local indication that an independent protective mechanism permits or inhibits ordinary motion. |
| Monotonic time | Elapsed-time value that does not move backward and is independent of civil-time adjustments. |
| Revision | Increasing counter identifying an atomic change to externally observable control state. |
| Snapshot | Consistent observation of runtime control state at a particular revision. |
| Topic | Named telemetry contract carrying one defined payload type. |
| Freshness | Maximum acceptable age of a measurement in the runtime’s monotonic clock domain. |
| Coordinate frame | Right-handed reference system used to interpret spatial quantities. |
| Qualification profile | Recorded hardware, software, configuration, load, and commissioning conditions for which conformance was tested. |
| Conforming boundary | Components and interfaces included in the assessed implementation and its responsibility for actuator effects. |

### 1.5 Abbreviations

| Abbreviation | Meaning |
|---|---|
| API | Application Programming Interface |
| HTTP | Hypertext Transfer Protocol |
| JSON | JavaScript Object Notation |
| REST | Representational State Transfer |
| SI | International System of Units |
| TLS | Transport Layer Security |
| UTC | Coordinated Universal Time |

### 1.6 Conformance keywords

The keywords **MUST**, **MUST NOT**, **REQUIRED**, **SHOULD**, **SHOULD NOT**, and **MAY** are interpreted according to RFC 2119 and RFC 8174 when written in uppercase.

Field tables, state tables, protocol rules, and numerical limits are normative unless explicitly identified as illustrative. Their obligations are allocated to the requirement identifiers in Section 5.

A **SHOULD** deviation requires a documented engineering justification in the conformance report. A **MUST** deviation prevents the affected conformance claim.

## 2. Conformance levels

The levels are cumulative. Basic conformance includes the complete motion-authority and stop contract; higher levels add observable timing qualification, event recovery, audit protection, and resilience testing.

| Level | Name | Required capability |
|---|---|---|
| 1 | Basic | Complete runtime, configuration, local execution, command, safety-boundary, security, and telemetry contracts. |
| 2 | Standard | Level 1 plus event replay, integrity-protected audit records, and measured timing qualification. |
| 3 | Advanced | Level 2 plus overload isolation and crash-recovery qualification. |

The discovery value `conformance_level` is an integer enumeration: `1` means Basic, `2` means Standard, and `3` means Advanced.

| Requirement IDs | Level 1 | Level 2 | Level 3 |
|---|---:|---:|---:|
| REQ-FUN-001 through REQ-FUN-007 | Required | Required | Required |
| REQ-FUN-008 | Optional | Required | Required |
| REQ-PER-001 | Required | Required | Required |
| REQ-PER-002 | Optional | Required | Required |
| REQ-PER-003 | Optional | Optional | Required |
| REQ-SAF-001 through REQ-SAF-004 | Required | Required | Required |
| REQ-SEC-001, REQ-SEC-002, REQ-SEC-004 | Required | Required | Required |
| REQ-SEC-003 | Optional | Required | Required |
| REQ-INT-001 through REQ-INT-003 | Required | Required | Required |
| REQ-ACC-001 | Conditional | Conditional | Conditional |
| REQ-RES-001 | Optional | Optional | Required |
| REQ-GOV-001 | Required | Required | Required |

REQ-ACC-001 applies when an operator interface is included in the conforming boundary. Absence of such an interface is reported as “not applicable,” with the assessed boundary identified.

An optional capability, when exposed under a standardized interface, MUST follow its specified semantics. A higher-level claim requires every requirement assigned to that level; implementing selected higher-level features does not establish that level.

## 3. Reference architecture / system model

The runtime comprises a management service, authority manager, local executor, telemetry broker, component supervisor, and driver adapters. These may share a process only where required fault isolation and deadline behavior remain demonstrable.

```text
 Authenticated operator / application
                  |
          HTTPS management API
                  |
       +----------v-----------+
       | Management service   |
       | Validation, revisions|
       | request deduplication|
       +----------+-----------+
                  |
       +----------v-----------+       Audit / event consumers
       | Authority manager    |-------------------->
       | Lease, epoch, expiry |
       +----------+-----------+
                  |
       +----------v-----------+       +----------------------+
       | Local executor       |<------| Component supervisor |
       | Periodic validation  |       | Health, dependencies |
       | Current setpoint     |       +----------------------+
       +----------+-----------+
                  |
       +----------v-----------+       +----------------------+
       | Driver adapters      |------>| Telemetry broker     |
       | Fencing and watchdogs|       | State and transforms |
       | Configured stop path |       +----------------------+
       +----------+-----------+
                  |
             Actuators
                  ^
                  |
       Independent protective mechanism
       and actuator-side stop capability
```

The management service parses and authenticates requests before control-state mutation. The authority manager serializes configuration, lifecycle, lease, and command transactions. The local executor rechecks authority immediately before each ordinary actuator handoff.

Driver adapters enforce the boot identifier, epoch, and command deadline at the execution boundary. They MUST prevent delayed device messages from restoring superseded authority. An adapter may implement this through device-supported fencing, connection-generation isolation, or another demonstrably equivalent mechanism.

The protective mechanism lies outside ordinary command ownership. Its stop indication cannot be suppressed by a lease owner or by API availability. A software snapshot reporting `FAULT` is not itself evidence that physical motion has stopped.

The telemetry broker aggregates driver and component observations. Acquisition timestamps describe the observations, not the later HTTP response time. Wall-clock timestamps MAY be recorded for human correlation but MUST NOT determine lease or command validity.

Ordinary motion commands and configured stopping actions use separate logical authorization paths. Revoking ordinary motion authority MUST leave the stopping path available. Stopping does not necessarily mean removing actuator power; supported loads may require continued holding torque or another commissioned behavior.

## 4. Data model

### 4.1 Common types and encoding

All identifiers and field names are case-sensitive.

| Type | Representation | Constraints |
|---|---|---|
| `Name` | JSON string | ASCII; pattern `[A-Za-z][A-Za-z0-9_.-]{0,63}`. |
| `Id128` | JSON string | Exactly 32 lowercase hexadecimal characters; generated with at least 128 bits of random input. |
| `Counter` | JSON string | Canonical unsigned decimal integer from `0` through `18446744073709551615`; no leading zeros except `0`. |
| `Time` | JSON string | Same encoding and range as `Counter`; nanoseconds since the start of the identified boot. |
| `Duration` | JSON integer | Nonnegative nanoseconds; field-specific limits apply. |
| `Real` | JSON number | Finite numeric value; NaN and infinities are prohibited. |
| `Text` | JSON string | Valid Unicode; at most 512 Unicode scalar values unless a stricter limit is stated. |

String encoding for counters and absolute nanosecond times avoids loss of precision in JSON implementations using binary floating-point numbers.

All listed object fields are required unless marked otherwise. `null` is permitted only where explicitly stated. Unknown fields, duplicate object member names, and duplicate identifiers within an identifier array MUST be rejected.

### 4.2 Runtime status

`RuntimeStatus` is returned by the runtime observation endpoint and embedded in control events.

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `spec_version` | string | — | Yes | Constant `1.0` | Negotiated specification version. |
| `runtime_id` | Name | — | Yes | Stable within installation | Managed runtime. |
| `robot_id` | Name | — | Yes | Stable within installation | Represented robot. |
| `boot_id` | Id128 | — | Yes | New for every incarnation | Clock and authority domain. |
| `revision` | Counter | — | Yes | Starts at `0`; strictly increases on control changes | Snapshot watermark. |
| `observed_at_ns` | Time | ns | Yes | Current boot domain | Snapshot observation time. |
| `state` | enum | — | Yes | Section 4.3 | Runtime lifecycle state. |
| `stop_status` | enum | — | Yes | Section 4.3 | Physical stop observation. |
| `protective_state` | enum | — | Yes | Section 4.3 | Protective-input observation. |
| `motion_permitted` | boolean | — | Yes | Derived under Section 5 | Permission for ordinary output. |
| `configuration_id` | Name or null | — | Yes | Installed configuration or null | Configuration identity. |
| `epoch` | Counter | — | Yes | Never decreases within a boot | Current fencing generation. |
| `lease` | Lease or null | — | Yes | At most one live lease | Current authority. |
| `current_command` | CommandRecord or null | — | Yes | Nonterminal command only | Current velocity setpoint. |
| `fault` | Fault or null | — | Yes | Non-null in `FAULT` | Latched fault information. |

Telemetry values can change without changing `revision`. Changes to lifecycle state, stop status, protective state, lease state, command status, fault records, or installed configuration MUST increment it.

### 4.3 Enumerations

The following are the complete value sets.

| Enumeration | Value | Meaning |
|---|---|---|
| Runtime state | `UNCONFIGURED` | Ordinary output inhibited; no installed configuration. |
|  | `CONFIGURING` | Configuration installed provisionally; dependency, driver, and stop checks running. |
|  | `INACTIVE` | Configuration ready; ordinary motion disabled. |
|  | `ACTIVE` | Runtime may accept motion authority and commands subject to guards. |
|  | `STOPPING` | Ordinary authority revoked; configured stop procedure running. |
|  | `FAULT` | Fault latched; ordinary authority revoked; stop confirmation may remain outstanding. |
|  | `FINALIZED` | Terminal state for this boot. |
| Stop status | `UNKNOWN` | Required stop or motion evidence is unavailable or insufficient. |
|  | `MOVING` | At least one measured joint velocity exceeds its stop threshold. |
|  | `STOPPED` | All stop-confirmation conditions are satisfied. |
| Protective state | `CLEAR` | Protective input permits ordinary operation. |
|  | `ASSERTED` | Protective input demands inhibition and stopping. |
|  | `UNKNOWN` | Protective input cannot be trusted or observed. |
| Component role | `DRIVER` | Implements actuator and feedback interaction. |
|  | `CONTROLLER` | Coordinates ordinary joint-output handoff. |
|  | `APPLICATION` | Performs higher-level computation without bypassing authority. |
|  | `MONITOR` | Produces supervisory or diagnostic observations. |
| Component health | `READY` | Component meets its declared operating contract. |
|  | `DEGRADED` | Component operates with a declared limitation. |
|  | `FAILED` | Component cannot meet its contract. |
|  | `UNKNOWN` | No sufficiently fresh health observation exists. |
| Joint type | `REVOLUTE` | Rotary joint with finite position limits. |
|  | `CONTINUOUS` | Rotary joint with unwrapped position and no position limits. |
|  | `PRISMATIC` | Linear joint with finite position limits. |
| Command mode | `JOINT_VELOCITY` | Signed joint-velocity targets maintained until replacement or termination. |
| Command status | `ACCEPTED` | Installed but not yet handed off. |
|  | `EXECUTING` | At least one valid execution handoff occurred. |
|  | `SUPERSEDED` | Replaced by a later accepted command. |
|  | `EXPIRED` | Its deadline ended while it was current. |
|  | `CANCELLED` | Terminated by lifecycle or authority revocation. |
|  | `FAILED` | Execution failed because of a driver, limit, health, or protective fault. |
| Lifecycle action | `ACTIVATE` | Request entry to `ACTIVE`. |
|  | `DEACTIVATE` | Request orderly stopping and entry to `INACTIVE`. |
|  | `RESET` | Clear an eligible fault and remove configuration. |
|  | `FINALIZE` | End this runtime incarnation’s managed operation. |
| Event cause | `CONFIGURATION` | Configuration installation or readiness changed. |
|  | `LIFECYCLE` | Lifecycle state changed. |
|  | `LEASE` | Lease or epoch changed. |
|  | `COMMAND` | Command identity or status changed. |
|  | `FAULT` | Fault record changed. |
|  | `STOP_STATUS` | Stop or protective observation changed. |
| Audit outcome | `ACCEPTED` | Authorized operation committed. |
|  | `REJECTED` | Operation was rejected without its requested mutation. |
|  | `INTERNAL` | Change originated inside the runtime. |

A terminal command status is `SUPERSEDED`, `EXPIRED`, `CANCELLED`, or `FAILED`. Terminal records are immutable.

### 4.4 Configuration

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `configuration_id` | Name | — | Yes | New identity for changed content | Configuration identity. |
| `robot_id` | Name | — | Yes | Matches runtime | Target robot. |
| `period_ns` | Duration | ns | Yes | Section 5.2 | Local control period, \(T\). |
| `command_timeout_ns` | Duration | ns | Yes | Section 5.2 | Maximum command validity, \(D\). |
| `lease_timeout_ns` | Duration | ns | Yes | Section 5.2 | Lease duration, \(L\). |
| `startup_timeout_ns` | Duration | ns | Yes | Section 5.2 | Configuration-readiness deadline. |
| `stop_timeout_ns` | Duration | ns | Yes | Section 5.2 | Commissioned stop-confirmation deadline. |
| `components` | Component array | — | Yes | 1–32 entries | Dependency and role declarations. |
| `joints` | Joint array | — | Yes | 1–64 entries | Entire controlled resource set. |
| `frames` | Frame array | — | Yes | 1–128 entries | Coordinate-frame tree. |

The configuration MUST contain exactly one `CONTROLLER`. That controller and every `DRIVER` referenced by a joint MUST be critical.

Configuration identity is content-bound: reusing an identifier for different configuration content within retained records is prohibited.

#### Component

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `component_id` | Name | — | Yes | Unique | Component identity. |
| `role` | enum | — | Yes | Component role | Functional responsibility. |
| `critical` | boolean | — | Yes | Required true for controlling components | Whether readiness gates motion. |
| `depends_on` | Name array | — | Yes | Existing components; no self-reference | Startup and failure dependencies. |

Dependencies MUST form a directed acyclic graph. Components start only after dependencies become `READY`. Shutdown follows reverse dependency order after ordinary motion has been fenced.

#### Joint

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `joint_id` | Name | — | Yes | Unique | Joint identity. |
| `type` | enum | — | Yes | Joint type | Motion category. |
| `driver_id` | Name | — | Yes | References a `DRIVER` | Owning adapter. |
| `lower_limit` | Real or null | m or rad | Yes | Finite for bounded joints | Lower allowed position. |
| `upper_limit` | Real or null | m or rad | Yes | Greater than lower limit | Upper allowed position. |
| `max_velocity` | Real | m/s or rad/s | Yes | Greater than zero | Maximum target magnitude. |
| `max_acceleration` | Real | m/s² or rad/s² | Yes | Greater than zero | Ordinary setpoint slew limit. |
| `stop_margin` | Real | m or rad | Yes | Nonnegative | Commissioned worst-case stopping displacement bound. |
| `stop_velocity` | Real | m/s or rad/s | Yes | Greater than zero; below `max_velocity` | Stop-observation threshold. |

For `CONTINUOUS` joints, both position limits MUST be null and `stop_margin` MUST be zero. Position MUST be reported as an unwrapped angle.

For other joints, both limits MUST be finite and `2 × stop_margin` MUST be less than the position range. The margin MUST include detection, communication, actuator response, stopping travel, and measurement uncertainty under the qualification profile.

`max_acceleration` limits changes in ordinary velocity setpoints. It is not a claim about guaranteed braking capability and MUST NOT be substituted for the commissioned stopping-displacement bound.

#### Frame

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `frame_id` | Name | — | Yes | Unique | Child-frame identity. |
| `parent_frame_id` | Name or null | — | Yes | Existing parent or root null | Tree parent. |
| `translation_m` | Real array | m | Yes | Exactly three elements | Child origin expressed in parent. |
| `rotation_xyzw` | Real array | dimensionless | Yes | Exactly four elements | Child-to-parent quaternion. |
| `dynamic` | boolean | — | Yes | Root false | Whether telemetry updates the transform. |

Exactly one root MUST exist, named `base`, with null parent, zero translation, identity quaternion `[0,0,0,1]`, and `dynamic: false`.

Frames MUST form a connected, acyclic tree. Quaternion norm MUST differ from one by no more than \(10^{-6}\). Invalid quaternions MUST be rejected rather than silently normalized.

For a point \(p_c\) in a child frame:

\[
p_p = R(q_{xyzw})p_c + t
\]

defines its coordinates in the parent frame.

### 4.5 Lease, command, and fault records

#### Lease

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `lease_id` | Id128 | — | Yes | Unique within boot | Lease identity. |
| `owner_id` | Name | — | Yes | Authenticated principal | Authorized owner. |
| `epoch` | Counter | — | Yes | Equals grant generation | Execution fence. |
| `granted_at_ns` | Time | ns | Yes | Grant time | Original acquisition time. |
| `expires_at_ns` | Time | ns | Yes | Strictly later than grant | Current lease deadline. |

Renewal preserves `lease_id`, `owner_id`, `epoch`, and `granted_at_ns`. It replaces `expires_at_ns` according to Section 7.

A lease identifier is not a bearer credential. Authentication and owner matching remain mandatory.

#### CommandRecord

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `command_id` | Id128 | — | Yes | Equals submitting `request_id` | Command identity. |
| `configuration_id` | Name | — | Yes | Matches installed configuration | Interpretation of joint targets. |
| `lease_id` | Id128 | — | Yes | Live lease at acceptance | Submitting authority. |
| `epoch` | Counter | — | Yes | Matches lease and runtime | Execution generation. |
| `mode` | enum | — | Yes | `JOINT_VELOCITY` | Command semantics. |
| `status` | enum | — | Yes | Command status | Observable progress. |
| `accepted_at_ns` | Time | ns | Yes | Transaction time | Acceptance timestamp. |
| `updated_at_ns` | Time | ns | Yes | At least acceptance time | Latest status change. |
| `expires_at_ns` | Time | ns | Yes | Section 6.3 | Absolute command deadline. |
| `targets` | Target array | joint-dependent | Yes | Every configured joint exactly once | Complete velocity setpoint. |

Each `Target` has two required fields:

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `joint_id` | Name | — | Yes | Configured joint | Target identity. |
| `velocity` | Real | m/s or rad/s | Yes | Absolute value no greater than `max_velocity` | Signed target velocity. |

Array order carries no joint identity. Implementations MUST resolve targets by `joint_id`.

#### Fault

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `codes` | enum array | — | Yes | Nonempty; unique | Latched fault causes. |
| `detected_at_ns` | Time | ns | Yes | Earliest cause in this fault episode | Detection time. |
| `detail` | Text | — | Yes | Human-readable; no credentials | Diagnostic explanation. |

Complete fault-code enumeration:

| Code | Meaning |
|---|---|
| `DATA_STALE` | Required feedback or critical health data exceeded its freshness limit. |
| `COMPONENT_FAILURE` | A critical component was not ready or failed. |
| `LIMIT_VIOLATION` | A measured limit or stopping-envelope condition was violated. |
| `WATCHDOG_EXPIRED` | The current command expired. |
| `LEASE_EXPIRED` | The current motion lease expired. |
| `PROTECTIVE_STOP` | Protective state became `ASSERTED` or `UNKNOWN`. |
| `STARTUP_TIMEOUT` | Configuration readiness was not established in time. |
| `STOP_TIMEOUT` | Stop confirmation was not obtained in time. |
| `DEADLINE_MISS` | A local execution deadline was missed. |
| `DRIVER_FAILURE` | Driver interaction or execution fencing failed. |
| `AUDIT_FAILURE` | Required audit persistence or integrity became unavailable. |

Additional causes append to `codes`; they do not erase earlier causes.

### 4.6 Telemetry

All telemetry uses this `Sample` envelope.

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `spec_version` | string | — | Yes | `1.0` | Payload version. |
| `runtime_id` | Name | — | Yes | Source runtime | Runtime identity. |
| `boot_id` | Id128 | — | Yes | Current incarnation | Timestamp domain. |
| `topic` | enum | — | Yes | Table below | Payload contract. |
| `sequence` | Counter | — | Yes | Per-topic; strictly increasing | Publication sequence. |
| `monotonic_ns` | Time | ns | Yes | Acquisition time | Oldest observation in payload. |
| `payload` | object | — | Yes | Topic-specific | Measurements. |

| Topic value | Payload | Freshness limit |
|---|---|---|
| `joint_state` | `{"joints": JointObservation[]}` | `2T` |
| `frame_transform` | `{"transforms": TransformObservation[]}` | `5T` for dynamic transforms |
| `component_health` | `{"components": HealthObservation[]}` | `2T` for critical components |

`JointObservation` fields are:

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `joint_id` | Name | — | Yes | Configured joint | Measurement identity. |
| `position` | Real | m or rad | Yes | Unwrapped for continuous joints | Measured position. |
| `velocity` | Real | m/s or rad/s | Yes | Signed | Measured velocity. |
| `effort` | Real or null | N or N·m | Yes | Null if unavailable | Measured or estimated effort. |

A joint-state payload MUST contain every configured joint exactly once. Its acquisition skew MUST not exceed `T`.

`TransformObservation` contains the same `frame_id`, `parent_frame_id`, `translation_m`, and `rotation_xyzw` fields as `Frame`, with identical constraints. It MUST contain every dynamic frame exactly once and MUST NOT change the configured parent relationship. An empty transform array is valid when no dynamic frames exist.

`HealthObservation` contains required fields `component_id: Name`, `health: component-health enum`, and `detail: Text`. Every configured component MUST appear exactly once.

The three telemetry topics use latest-value delivery. A slow subscriber may miss intermediate samples; sequence jumps expose that loss. A missing or future-dated sample is not fresh. A driver using another clock MUST translate acquisition time into the runtime clock and include conversion uncertainty when establishing freshness.

### 4.7 JSON Schema excerpt

The following excerpt defines the complete structural shape of a command request. Cross-field authority, timing, identifier uniqueness, and physical-limit checks remain mandatory.

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "WIA-ROB-020 v1.0 CommandRequest",
  "type": "object",
  "additionalProperties": false,
  "required": [
    "boot_id",
    "request_id",
    "expected_revision",
    "expires_at_ns",
    "configuration_id",
    "lease_id",
    "epoch",
    "mode",
    "targets"
  ],
  "properties": {
    "boot_id": { "$ref": "#/$defs/id128" },
    "request_id": { "$ref": "#/$defs/id128" },
    "expected_revision": { "$ref": "#/$defs/counter" },
    "expires_at_ns": { "$ref": "#/$defs/counter" },
    "configuration_id": { "$ref": "#/$defs/name" },
    "lease_id": { "$ref": "#/$defs/id128" },
    "epoch": { "$ref": "#/$defs/counter" },
    "mode": { "const": "JOINT_VELOCITY" },
    "targets": {
      "type": "array",
      "minItems": 1,
      "maxItems": 64,
      "items": {
        "type": "object",
        "additionalProperties": false,
        "required": ["joint_id", "velocity"],
        "properties": {
          "joint_id": { "$ref": "#/$defs/name" },
          "velocity": { "type": "number" }
        }
      }
    }
  },
  "$defs": {
    "id128": {
      "type": "string",
      "pattern": "^[0-9a-f]{32}$"
    },
    "counter": {
      "type": "string",
      "pattern": "^(0|[1-9][0-9]{0,19})$"
    },
    "name": {
      "type": "string",
      "pattern": "^[A-Za-z][A-Za-z0-9_.-]{0,63}$"
    }
  }
}
```

The decimal-string pattern does not independently enforce the unsigned 64-bit maximum. Semantic validation MUST enforce the bound.

### 4.8 Complete illustrative configuration

The following is a complete, valid example configuration. Its physical values are illustrative commissioning choices, not universal safe limits.

```json
{
  "configuration_id": "cfg-slide-1",
  "robot_id": "robot-demo",
  "period_ns": 10000000,
  "command_timeout_ns": 100000000,
  "lease_timeout_ns": 1000000000,
  "startup_timeout_ns": 5000000000,
  "stop_timeout_ns": 1000000000,
  "components": [
    {
      "component_id": "drive",
      "role": "DRIVER",
      "critical": true,
      "depends_on": []
    },
    {
      "component_id": "control",
      "role": "CONTROLLER",
      "critical": true,
      "depends_on": ["drive"]
    },
    {
      "component_id": "monitor",
      "role": "MONITOR",
      "critical": false,
      "depends_on": ["drive"]
    }
  ],
  "joints": [
    {
      "joint_id": "slide",
      "type": "PRISMATIC",
      "driver_id": "drive",
      "lower_limit": 0.0,
      "upper_limit": 1.0,
      "max_velocity": 0.2,
      "max_acceleration": 0.5,
      "stop_margin": 0.05,
      "stop_velocity": 0.001
    }
  ],
  "frames": [
    {
      "frame_id": "base",
      "parent_frame_id": null,
      "translation_m": [0.0, 0.0, 0.0],
      "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
      "dynamic": false
    },
    {
      "frame_id": "carriage",
      "parent_frame_id": "base",
      "translation_m": [0.5, 0.0, 0.0],
      "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
      "dynamic": true
    }
  ]
}
```

## 5. Requirements and thresholds

### 5.1 Functional requirements

**REQ-FUN-001 — Lifecycle.**  
The runtime MUST implement the state machine in Section 7. Every ordinary actuator output MUST be inhibited outside `ACTIVE`. Configured stopping and holding actions remain available in other states where required.

**REQ-FUN-002 — Configuration integrity.**  
Configuration validation MUST cover all structural, reference, graph, unit, limit, and commissioning constraints. Installation MUST be atomic. A failed structural validation MUST leave the prior state unchanged. Readiness failure after accepted installation MUST enter `FAULT`.

**REQ-FUN-003 — Driver contract.**  
Each driver MUST support feedback acquisition, ordinary velocity handoff, authority fencing, stop initiation, and stop-completion observation. The semantic operations are:

- `read_state`: return the adapter’s joint observations and acquisition time.
- `write_velocity`: accept boot identifier, epoch, command identifier, absolute deadline, and joint targets.
- `begin_stop`: fence ordinary work and initiate the commissioned stop procedure.
- `poll_stop`: report whether execution-side fencing and the adapter’s stopping action are complete.

These names describe required semantics, not a mandated programming-language ABI. Partial multi-driver execution cannot be rolled back; any failed handoff MUST fault the runtime and initiate stopping.

**REQ-FUN-004 — Exclusive authority.**  
At most one lease may exist per runtime. Acquisition requires `ACTIVE`, `STOPPED`, and no current lease. Renewal MAY occur during motion, but only by the current owner before expiry. Lease expiry MUST revoke authority and initiate stopping.

**REQ-FUN-005 — Velocity command semantics.**  
A command MUST include every configured joint exactly once. The runtime MUST hold only one current command and MUST NOT maintain an ordinary motion-command queue. Accepting a new command atomically marks the previous current command `SUPERSEDED`.

Targets MUST NOT be silently clipped. Accepted target changes MUST respect `max_acceleration` during ordinary output generation. Superseded, expired, cancelled, and failed commands MUST never become current again.

**REQ-FUN-006 — Atomic requests and retries.**  
Section 6 transaction ordering, revision checks, request deadlines, deduplication, and response retention MUST be implemented. Atomic acceptance does not imply simultaneous mechanical response or physical rollback.

**REQ-FUN-007 — Discovery and observation.**  
The runtime MUST expose its identity, conformance level, configuration, current state, supported telemetry contracts, and retained command records. Observation MUST remain available during `FAULT` and `FINALIZED`, subject to authentication and service availability.

**REQ-FUN-008 — Recoverable control events.**  
Every control revision MUST produce exactly one event envelope. All changes belonging to that revision MUST be included together. The implementation MUST retain at least the latest 4,096 events for the current boot and implement explicit cursor-expiry behavior.

### 5.2 Performance and timing

The following values are requirements selected by this standard. They define a bounded supervisory joint-control profile; they are not claims about the capabilities or prevalence of robots.

| Quantity | Normative value or rule | Engineering rationale |
|---|---|---|
| Control period \(T\) | 1,000,000–20,000,000 ns | Bounds local validation and watchdog reaction while permitting different execution platforms. |
| Maximum command validity \(D\) | `5T ≤ D ≤ 1,000,000,000 ns` | Allows several execution cycles while bounding unattended target persistence. |
| Lease duration \(L\) | `2D ≤ L ≤ 10,000,000,000 ns` | Separates short command freshness from longer authority renewal. |
| Startup timeout | `3T` through 60,000,000,000 ns | Permits stop evidence while bounding incomplete startup. |
| Stop timeout | `T` through 60,000,000,000 ns | Requires a finite commissioned observation deadline; does not establish safe stopping time. |
| General mutation admission window | At most 5,000,000,000 ns | Bounds delayed management operations. |
| Accepted-command remaining validity | At least `2T`, at most `D` | Allows a next-cycle handoff without accepting long-lived motion. |
| Retry and command-record retention | Through request or command deadline plus 60,000,000,000 ns | Provides bounded recovery after a lost response. |
| Request body limit | 65,536 bytes | Bounds parser and request-storage work. |
| Stop qualification | Three consecutive qualifying control cycles | Requires repeated fresh observations rather than one transient sample. |

**REQ-PER-001 — Local timing.**  
For scheduled release \(s_k\), ordinary output handoff MUST finish no later than \(s_k + T\). A newly accepted command MUST be considered at the next scheduled release. Scheduler delay is included in the deadline.

Expiration, protective-input changes, and missed deadlines MUST cause local stop initiation no later than one additional period after the relevant boundary. Enforcement MUST remain effective if the management service stalls.

A monotonic-clock discontinuity or loss of authority-process continuity MUST create a new `boot_id`, invalidate prior authority, and inhibit ordinary output. Suspend and resume MUST NOT preserve a motion lease. Counter exhaustion MUST similarly end the incarnation before wraparound.

**REQ-PER-002 — Standard timing qualification.**  
Level 2 MUST complete at least 100,000 consecutive control periods with zero missed handoff deadlines under the baseline workload in Section 8. Valid `GET` requests in that workload MUST have server processing latency no greater than 250 ms, measured from receipt of the complete request to queuing the complete response.

**REQ-PER-003 — Advanced overload isolation.**  
Level 3 MUST complete at least 1,000,000 consecutive baseline periods with zero missed handoff deadlines. During a separate run of at least 100,000 periods, observation traffic MUST be increased to twenty times the baseline observation rate. The service MAY reject excess requests with `429`, but control deadlines, command expiry, lease expiry, and protective stopping MUST remain within their specified bounds.

These finite runs establish observed performance for the qualification profile. They do not prove universal worst-case execution time.

### 5.3 Safety-related runtime behavior

**REQ-SAF-001 — Commissioned protection boundary.**  
The qualification profile MUST identify the protective input, actuator-side watchdog, configured stop procedure, holding behavior, stopping-displacement evidence, and response when stopping cannot be confirmed. Ordinary REST availability MUST NOT be the sole protective mechanism.

The commissioning evidence MUST cover the declared load, velocity, supply, actuator, and measurement conditions. A configuration outside that evidence MUST NOT be activated.

**REQ-SAF-002 — Stop initiation and confirmation.**  
Entering `STOPPING` or `FAULT` MUST atomically revoke the lease, advance the epoch, invalidate ordinary pending work, and invoke each affected driver’s stop path.

`STOPPED` requires:

1. Every driver positively confirms that earlier ordinary commands are fenced at the execution side.
2. Every driver reports its configured stopping action complete.
3. Every joint has three consecutive fresh, post-stop observations with absolute velocity no greater than `stop_velocity`.
4. The observations span three distinct control cycles and are not repeated readings of one cached sample.

Missing evidence produces `UNKNOWN`, unless fresh velocity evidence establishes `MOVING`. Failure to confirm by `stop_timeout_ns` MUST latch `STOP_TIMEOUT` and invoke the commissioned escalation response. `FAULT` MUST remain latched.

**REQ-SAF-003 — Readiness and watchdogs.**  
`motion_permitted` is true only when all of the following hold:

- State is `ACTIVE`.
- Protective state is `CLEAR`.
- Every critical component is `READY` with fresh health data.
- Joint feedback is fresh.
- A live lease matches the current boot and epoch.
- A current command is valid and unexpired.
- Applicable joint guards are satisfied.

Command expiry MUST mark the current command `EXPIRED`, latch `WATCHDOG_EXPIRED`, and begin stopping. Lease expiry MUST cancel any current command, latch `LEASE_EXPIRED`, and begin stopping. If both expire together, both fault codes MUST be recorded and the command status MUST be `EXPIRED`.

Loss of critical readiness, feedback freshness, protective permission, or driver validity MUST fault the runtime before further ordinary output.

**REQ-SAF-004 — Joint bounds and stopping envelope.**  
For a bounded joint at measured position \(q\), ordinary positive motion requires:

\[
q + \text{stop_margin} < \text{upper_limit}
\]

Ordinary negative motion requires:

\[
q - \text{stop_margin} > \text{lower_limit}
\]

These guards MUST be checked against both the requested direction and the measured direction of motion. Reaching a guard boundary while moving toward that limit MUST initiate stopping. A measured position outside the configured interval MUST fault the runtime.

An inward request from a stationary position near a limit MAY be accepted if all other guards hold. A zero target does not waive measured-motion checks. These scalar guards do not provide collision avoidance or account for every coupled mechanical hazard.

### 5.4 Security and privacy

**REQ-SEC-001 — Authentication and authorization.**  
Network API access MUST use TLS 1.3 or a later explicitly assessed compatible transport profile. Every request MUST resolve to an authenticated principal. Certificate, token, or equivalent credential provisioning MUST be documented.

The complete role enumeration is:

| Role | Permissions |
|---|---|
| `OBSERVER` | Read authorized discovery, status, configuration, topics, samples, commands, and events. |
| `OPERATOR` | Observer permissions; acquire, renew, and release its own lease; submit its own commands; request activation or deactivation. |
| `ADMINISTRATOR` | Operator permissions; install configuration, reset faults, finalize runtimes, and request deactivation regardless of lease ownership. |

Roles do not permit one principal to renew or submit commands using another principal’s lease. Administrative intervention stops existing authority; it does not impersonate the owner.

**REQ-SEC-002 — Input and resource isolation.**  
The runtime MUST reject malformed JSON, unsupported versions, unknown fields, oversized bodies, invalid references, invalid numeric values, and unauthorized resource access before control mutation.

Parser, connection, and storage limits MUST be declared in the qualification profile. Exhausted capacity MUST produce a defined rejection; it MUST NOT cause silent command loss, unbounded queuing, or eviction of a still-required deduplication record.

**REQ-SEC-003 — Audit integrity.**  
Level 2 and Level 3 MUST retain integrity-protected records of configuration changes, lifecycle changes, lease changes, command acceptance and terminal outcomes, faults, and denied mutation attempts.

Audit records MUST contain the fields in Section 8.4. Integrity verification MUST use an authenticated checkpoint or equivalent trust anchor outside ordinary runtime write authority. A hash stored only beside rewritable records is insufficient.

If required audit persistence fails, new motion authority and commands MUST be rejected, `AUDIT_FAILURE` MUST be latched, and stopping MUST begin. Protective stopping MUST proceed even when its audit write cannot complete.

**REQ-SEC-004 — Privacy and retention.**  
Runtime records MUST identify actors through provisioned principal identifiers. They MUST NOT require human names, contact details, or raw credentials.

The qualification profile MUST declare retention and authorized access for telemetry, command records, and audit records. The minimum operational retention in Section 5.2 cannot be shortened while the boot remains active. Longer retention MUST have an identified operational purpose. Expired records MUST be removed according to the declared policy, including exported copies under the implementation’s control.

### 5.5 Interoperability

**REQ-INT-001 — Wire encoding and versioning.**  
JSON MUST be UTF-8 and follow RFC 8259. API version negotiation, exact field spelling, decimal-string encoding, and error behavior MUST follow Sections 4 and 6. Unit conversion MUST occur before values enter standardized fields.

**REQ-INT-002 — Physical conventions.**  
Linear quantities MUST use metres, rotary quantities radians, time seconds or explicitly named nanoseconds, force newtons, and torque newton-metres.

Frames MUST be right-handed. The robot’s `base` frame uses positive X forward, positive Y left, and positive Z up; commissioning MUST identify those directions for robots without an obvious front. Quaternion order and transform direction MUST follow Section 4.4.

**REQ-INT-003 — Telemetry semantics.**  
Sample identity, acquisition time, sequence, completeness, freshness, frame relationships, and latest-value delivery MUST follow Section 4.6. Stale data MUST remain distinguishable from valid zero values. Configuration transforms for dynamic frames MUST NOT be presented as fresh measurements.

### 5.6 Accessibility

**REQ-ACC-001 — Operator interpretation.**  
An included operator interface MUST present lifecycle state, motion permission, stop status, and fault codes as text or equivalent accessible labels. It MUST distinguish “stop requested” from “stop confirmed.”

Color or animation MUST NOT be the sole indication of motion or fault state. Joint values MUST identify the joint and unit. Actions that acquire authority, activate motion capability, or reset faults MUST be keyboard operable when the interface supports keyboard input.

### 5.7 Resilience and governance

**REQ-RES-001 — Crash recovery.**  
Level 3 MUST demonstrate process termination and restart during motion without automatic authority restoration. Durable records MUST permit identification of previously accepted commands whose physical outcome is uncertain.

Recovery MUST establish a new boot identifier, fence old device traffic, begin with ordinary output inhibited, and require configuration and authority establishment again. An uncertain pre-crash outcome MUST be reported as an audit incident; it MUST NOT be relabelled as successful execution.

**REQ-GOV-001 — Assessment traceability.**  
Conformance claims MUST identify their qualification profile, applicable requirement set, test results, software and hardware identities, limitations, assessor, and validity conditions according to Sections 8 and 9.

## 6. Interfaces and API

### 6.1 Transport and paths

All standardized paths begin with `/v1`. Every request and response MUST carry:

```text
WIA-Spec-Version: 1.0
```

JSON bodies use `Content-Type: application/json`. Compressed request bodies are not supported in version 1.0.

The path variable `{runtime_id}` is a `Name`. `{lease_id}` and `{command_id}` are `Id128` values. `{topic}` is one of the three topic values in Section 4.6.

| Method | Path | Purpose | Request JSON | Success response JSON |
|---|---|---|---|---|
| GET | `/v1/runtimes` | Discover authorized runtimes | None | `DiscoveryResponse` |
| GET | `/v1/runtimes/{runtime_id}` | Read atomic control snapshot | None | `RuntimeStatus` |
| GET | `/v1/runtimes/{runtime_id}/configuration` | Read installed configuration | None | `Configuration` |
| PUT | `/v1/runtimes/{runtime_id}/configuration` | Install configuration | `ConfigureRequest` | `MutationResponse<RuntimeStatus>` |
| POST | `/v1/runtimes/{runtime_id}/lifecycle` | Request lifecycle action | `LifecycleRequest` | `MutationResponse<RuntimeStatus>` |
| POST | `/v1/runtimes/{runtime_id}/leases` | Acquire authority | `AcquireLeaseRequest` | `MutationResponse<Lease>` |
| POST | `/v1/runtimes/{runtime_id}/leases/{lease_id}/renew` | Renew owned authority | `RenewLeaseRequest` | `MutationResponse<Lease>` |
| POST | `/v1/runtimes/{runtime_id}/leases/{lease_id}/release` | Release owned authority and stop | `ReleaseLeaseRequest` | `MutationResponse<RuntimeStatus>` |
| POST | `/v1/runtimes/{runtime_id}/commands` | Submit complete velocity target | `CommandRequest` | `MutationResponse<CommandRecord>` |
| GET | `/v1/runtimes/{runtime_id}/commands/{command_id}` | Read retained command status | None | `CommandRecord` |
| GET | `/v1/runtimes/{runtime_id}/topics` | Discover telemetry contracts | None | `TopicResponse` |
| GET | `/v1/runtimes/{runtime_id}/topics/{topic}/sample` | Read latest sample | None | `Sample` |
| GET | `/v1/runtimes/{runtime_id}/events` | Read bounded control-event replay | None | `EventPage` |

### 6.2 Request and response objects

Every mutating request contains the following fields.

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `boot_id` | Id128 | — | Yes | Current boot | Prevents cross-boot requests. |
| `request_id` | Id128 | — | Yes | New for each logical request | Retry identity. |
| `expected_revision` | Counter | — | Yes | Current revision at commit | Optimistic concurrency guard. |
| `expires_at_ns` | Time | ns | Yes | Future; admission limits apply | Latest admissible execution time. |

Additional required fields are:

| Request type | Additional fields |
|---|---|
| `ConfigureRequest` | `configuration: Configuration` |
| `LifecycleRequest` | `action: lifecycle-action enum` |
| `AcquireLeaseRequest` | None |
| `RenewLeaseRequest` | `epoch: Counter` |
| `ReleaseLeaseRequest` | `epoch: Counter` |
| `CommandRequest` | `configuration_id: Name`, `lease_id: Id128`, `epoch: Counter`, `mode: JOINT_VELOCITY`, `targets: Target[]` |

For commands, `expires_at_ns` is both the admission deadline and the continued-motion deadline. Other mutations use it only as an admission deadline.

`MutationResponse<T>` has these required fields:

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `boot_id` | Id128 | — | Yes | Acceptance boot | Incarnation identity. |
| `request_id` | Id128 | — | Yes | Echoed request identity | Correlation. |
| `revision` | Counter | — | Yes | Resulting committed revision | Transaction watermark. |
| `accepted_at_ns` | Time | ns | Yes | Commit time | Acceptance timestamp. |
| `replayed` | boolean | — | Yes | True only for duplicate replay | Retry indication. |
| `result` | T | — | Yes | Endpoint-specific object | Original accepted result. |

A replayed result describes the original transaction and may be older than current state. Clients MUST read current status before assuming that a returned lease or command remains valid.

`DiscoveryResponse` contains `spec_version: "1.0"` and `runtimes`, an array of objects containing `runtime_id`, `robot_id`, and `conformance_level`.

`TopicResponse` contains `runtime_id`, `boot_id`, and `topics`. Each topic descriptor contains:

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `topic` | enum | — | Yes | Telemetry topic | Contract identity. |
| `delivery` | string | — | Yes | Constant `LATEST` | Latest-value delivery. |
| `max_age_ns` | Duration or null | ns | Yes | Section 4.6; null without configuration | Configured freshness limit. |

### 6.3 Mutation processing

The runtime MUST process a mutation in this order:

1. Apply bounded transport parsing and authenticate the principal.
2. Authorize the endpoint and target resource.
3. Validate the request’s structure and version.
4. Reject a mismatched boot identifier.
5. Look up an accepted duplicate using principal, boot identifier, and request identifier.
6. For an identical duplicate, return the stored result without repeating effects.
7. For the same retained identity with different method, path, or parsed JSON content, return `REQUEST_ID_CONFLICT`.
8. Check admission deadline, revision, lifecycle, authority, and physical guards.
9. Commit the entire transaction, its revision, and its deduplication record atomically.
10. Return the result.

Object-member order and insignificant JSON whitespace do not change request identity. Array order remains significant. Numeric values are compared by their parsed mathematical value within the supported numeric domain.

A general request is admissible only when:

\[
r < \text{expires_at_ns} \le r + 5{,}000{,}000{,}000
\]

where \(r\) is the commit-time monotonic value.

A command additionally requires:

\[
r + 2T \le \text{expires_at_ns} \le r + D
\]

and its deadline MUST NOT exceed the current lease deadline.

Accepted request records MUST remain available through their request deadline plus the retention interval. After removal, an unchanged original request is necessarily expired and MUST NOT execute again. Reuse detection for modified requests is guaranteed only during retention; clients MUST never intentionally reuse a request identifier within a boot.

A revision conflict has no requested control effect. A client wishing to resubmit modified content MUST observe current state and create a new request identifier.

### 6.4 Status and error codes

Successful reads, lease operations, and synchronous lifecycle actions return `200`. Accepted configuration, command submission, and actions entering `STOPPING` return `202`. Acceptance does not imply readiness, actuator movement, or stop completion.

Errors use:

```json
{
  "error": {
    "code": "STATE_CONFLICT",
    "message": "Activation requires the INACTIVE state.",
    "request_id": "11111111111111111111111111111111",
    "boot_id": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
    "revision": "4"
  }
}
```

This error document is illustrative.

`code` and `message` are required. `request_id`, `boot_id`, and `revision` are required but MAY be null when unavailable or inappropriate to disclose.

| HTTP status | Error code | Meaning |
|---:|---|---|
| 400 | `INVALID_REQUEST` | Malformed shape, duplicate fields, invalid query, or invalid scalar representation. |
| 401 | `UNAUTHENTICATED` | Authentication absent or invalid. |
| 403 | `FORBIDDEN` | Principal lacks the required permission or lease ownership. |
| 404 | `NOT_FOUND` | Authorized resource does not exist or its retained record has expired. |
| 409 | `BOOT_MISMATCH` | Request identifies another incarnation. |
| 409 | `STALE_REVISION` | Expected revision does not match. |
| 409 | `STATE_CONFLICT` | Operation is invalid in the current lifecycle or resource state. |
| 409 | `LEASE_CONFLICT` | Lease, epoch, or exclusive-ownership condition fails. |
| 409 | `REQUEST_ID_CONFLICT` | Retained request identifier was reused with different content. |
| 409 | `DEADLINE_EXPIRED` | Request is no longer admissible. |
| 410 | `CURSOR_EXPIRED` | Requested event history is no longer retained. |
| 413 | `PAYLOAD_TOO_LARGE` | Request body exceeds the limit. |
| 415 | `UNSUPPORTED_MEDIA_TYPE` | Unsupported body representation or compression. |
| 422 | `VALIDATION_FAILED` | Well-formed content violates configuration, timing-window, frame, or joint constraints. |
| 426 | `UNSUPPORTED_VERSION` | Requested specification version is unsupported. |
| 429 | `RATE_LIMITED` | Declared service capacity is exhausted. |
| 501 | `UNSUPPORTED_FEATURE` | Optional standardized capability is not implemented. |
| 503 | `NOT_READY` | Required sample or service is unavailable. |
| 503 | `AUDIT_UNAVAILABLE` | Required audit persistence cannot accept the operation. |
| 500 | `INTERNAL_ERROR` | Unexpected service failure; outcome requires observation or safe retry. |

An unavailable configuration returns `404`. An unavailable current telemetry sample returns `503`.

### 6.5 Control events

An event has the following required fields:

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `boot_id` | Id128 | — | Yes | Current boot | Stream incarnation. |
| `event_seq` | Counter | — | Yes | Equals resulting runtime revision | Contiguous event sequence. |
| `monotonic_ns` | Time | ns | Yes | Commit time | Change timestamp. |
| `causes` | event-cause array | — | Yes | Nonempty; unique | Changed subsystems. |
| `runtime` | RuntimeStatus | — | Yes | Resulting control state | Atomic post-change snapshot. |
| `command_updates` | CommandRecord array | — | Yes | Every command changed by transaction | Includes terminal outcomes. |

The event endpoint requires query parameters `boot_id` and `after`. Optional `limit` is an integer from 1 through 100, defaulting to 100.

`EventPage` contains required fields `boot_id`, `after`, `next_after`, and `events`. The cursor fields are `Counter` values. `next_after` is the last returned event sequence, or the requested `after` if no event is returned.

A cursor greater than the current revision returns `INVALID_REQUEST`. An unavailable earlier cursor returns `CURSOR_EXPIRED`. Boot mismatch is never treated as an empty page.

Recovery consists of obtaining a snapshot with revision \(R\), then requesting events after \(R\). If those events expire before retrieval, the client repeats snapshot recovery. This restores current control state; it does not reconstruct already-lost historical events.

### 6.6 Versioning rules

`/v1` identifies the major interface family. `WIA-Spec-Version` identifies the exact negotiated contract.

Version 1.0 receivers reject unknown fields. A future minor version may add optional capabilities only through explicit negotiation; a peer negotiating `1.0` MUST continue receiving exactly the 1.0 contract.

Changes to existing field meaning, unit, command authority, state semantics, or error meaning require a new major version. Private extensions MUST use separate interfaces and MUST NOT alter standardized payloads.

## 7. Protocol and lifecycle

### 7.1 Runtime state machine

```text
UNCONFIGURED -- accepted configuration --> CONFIGURING
CONFIGURING  -- readiness and stop proof --> INACTIVE
INACTIVE     -- ACTIVATE -----------------> ACTIVE
ACTIVE       -- DEACTIVATE / release -----> STOPPING
STOPPING     -- stop confirmed -----------> INACTIVE

Any operational state -- detected fault --> FAULT
FAULT -- eligible RESET ------------------> UNCONFIGURED

UNCONFIGURED / INACTIVE -- FINALIZE ------> FINALIZED
```

“Operational state” means every state except `FINALIZED`.

| Current state | Trigger | Required guard | Result |
|---|---|---|---|
| `UNCONFIGURED` | Configuration PUT | Valid administrator request and commissioning profile | Install provisionally; enter `CONFIGURING`. |
| `CONFIGURING` | Internal readiness completion | Dependencies ready; drivers fenced; fresh state; `STOPPED`; protective input clear | Enter `INACTIVE`. |
| `CONFIGURING` | Startup deadline or readiness failure | None | Enter `FAULT`. |
| `INACTIVE` | `ACTIVATE` | Fresh required data, critical readiness, protective clear, stop confirmed | Enter `ACTIVE` without a lease. |
| `ACTIVE` | `DEACTIVATE` | Authorized operator or administrator | Revoke authority; enter `STOPPING`. |
| `ACTIVE` | Lease release | Current owner and epoch | Revoke authority; enter `STOPPING`. |
| `STOPPING` | Stop confirmation | Section 5.3 satisfied | Enter `INACTIVE`. |
| Any operational state | Fault | Detected applicable cause | Enter or remain `FAULT`; initiate or continue stopping. |
| `FAULT` | `RESET` | Administrator; causes corrected; protective clear; stop confirmed; required audit restored | Remove configuration and fault latch; enter `UNCONFIGURED`. |
| `UNCONFIGURED` or `INACTIVE` | `FINALIZE` | Administrator; no authority; ordinary output fenced; stop confirmed | Enter `FINALIZED`. |
| `FINALIZED` | Any mutation | None | Reject with `STATE_CONFLICT`. |

In an unconfigured runtime, stop confirmation depends on the commissioned adapter supervision available before configuration. If that supervision cannot establish stop evidence, `stop_status` remains `UNKNOWN`; finalization requiring stop confirmation is unavailable.

Illegal transitions MUST NOT partially perform the requested operation. A detected physical fault still takes precedence over request rejection.

### 7.2 Startup and configuration sequence

1. Establish a new boot identifier and reset revision and epoch to `0`.
2. Inhibit ordinary output and invalidate all prior driver sessions.
3. Begin commissioned stop supervision; initialize lifecycle as `UNCONFIGURED`.
4. Authenticate and validate a configuration request.
5. Atomically install the configuration and enter `CONFIGURING`.
6. Start components in dependency order while keeping ordinary output inhibited.
7. Acquire fresh feedback and obtain stop confirmation.
8. Enter `INACTIVE` only when all readiness guards hold.
9. Require a separate activation request before accepting a lease.

Readiness does not imply motion permission.

### 7.3 Lease acquisition and renewal

Acquisition in `ACTIVE` with `STOPPED` advances the epoch and creates a lease expiring at acceptance time plus `lease_timeout_ns`.

Renewal requires the same authenticated owner, lease identifier, and epoch. It sets expiry to renewal acceptance time plus `lease_timeout_ns`. It does not extend an existing command’s deadline.

A renewal received at or after lease expiry MUST fail and MUST NOT resurrect the lease. A fresh acquisition cannot occur until stopping is confirmed and the lifecycle again permits acquisition.

### 7.4 Command execution

1. The owner observes current boot, revision, and server monotonic time.
2. It submits a complete target set with a bounded absolute deadline.
3. The runtime validates the entire request atomically.
4. The previous current command becomes `SUPERSEDED`.
5. The new command becomes `ACCEPTED`.
6. At the next local release, the executor rechecks all motion guards.
7. It applies the ordinary acceleration constraint and hands off targets.
8. Successful first handoff changes status to `EXECUTING`.
9. Each later cycle rechecks deadline, lease, fencing, feedback, and limits.

A velocity command has no successful “completed” terminal state. It remains current until superseded or terminated. Reaching zero velocity does not release authority.

### 7.5 Stopping and faults

Explicit deactivation and lease release enter `STOPPING`. Expiry and detected faults enter `FAULT` directly while invoking the same commissioned stop procedure.

Authority revocation is immediate at the control transaction boundary. Physical stopping follows the commissioned mechanism and is observed independently.

The stop deadline begins when the stop procedure is first initiated for the episode. Additional fault causes MUST NOT restart that deadline. A `STOP_TIMEOUT` remains latched even if standstill is later observed.

Reset never restores the prior command, lease, or configuration. The epoch remains monotonically increasing within the boot.

### 7.6 Reconnection and restart

A reconnecting client MUST first read the current boot identifier. If unchanged, it may inspect retained command records or retry an identical accepted request. It MUST NOT infer continued motion authority from a cached success response.

If the boot identifier changed, the client MUST discard cached leases, epochs, deadlines, and revisions. All required setup and authority establishment must occur again.

## 8. Testing and conformance procedures

### 8.1 Test environment

Testing MUST identify the actual runtime build, operating environment, processors, actuator adapters, firmware, protective mechanism, and network topology.

The test environment MUST provide:

- A physical robot or hardware-in-the-loop arrangement with observable actuator-boundary behavior.
- Independent timing capture for scheduled releases, handoffs, fencing, and stop initiation.
- Controlled injection of stale feedback, device failures, delayed messages, process termination, and protective-input changes.
- Authenticated clients representing each role and an unauthorized client.
- Capture of requests, responses, revisions, command records, and applicable audit records.

Pure simulation MAY establish parser and state-machine behavior. It cannot by itself establish physical stopping margins, device fencing, or actuator watchdog behavior. A software-only assessment MUST identify the adapter simulator as its boundary and MUST NOT claim qualification of an attached physical robot.

Timing measurement uncertainty MUST be no greater than `T/10`. A measured result whose uncertainty crosses a required limit is not a pass.

The baseline workload consists of one owner submitting commands every `D/2`, renewing the lease every `L/2`, and one observer making a combined total of 20 status or sample requests per second. The owner may perform additional observations needed for revision handling. Commands MUST use physically admissible motion or a qualified actuator emulator.

### 8.2 Test cases

| ID | Requirement | Method | Pass criteria |
|---|---|---|---|
| TC-001 | REQ-FUN-001 | Exercise every allowed transition and attempt every prohibited transition. | State, guards, revocation, and terminal behavior match Section 7; no ordinary output outside `ACTIVE`. |
| TC-002 | REQ-FUN-002 | Submit cyclic dependencies, missing drivers, duplicate joints, invalid limits, invalid frames, and a valid configuration. | Invalid configurations have no installation effect; valid startup reaches the specified readiness result. |
| TC-003 | REQ-FUN-003 | Delay an old device command and fail one adapter during a multi-driver handoff. | Old authority is rejected; partial execution is reported through fault handling and stopping. |
| TC-004 | REQ-FUN-004 | Race two acquisitions; renew during motion; renew after expiry; attempt owner substitution. | Only one acquisition succeeds; valid moving renewal succeeds; expired or foreign authority is rejected. |
| TC-005 | REQ-FUN-005 | Submit missing, duplicate, excessive, and valid targets; supersede an executing command. | Atomic rejection or acceptance; slew limits hold; superseded work never reappears. |
| TC-006 | REQ-FUN-006 | Drop responses; replay identical requests; alter retained request content; race revisions; retry after retention. | No duplicate effect; conflicts have defined errors; expired original requests never execute again. |
| TC-007 | REQ-FUN-007 | Read discovery, configuration, status, and commands through normal, faulted, and finalized states. | Authorized observations match retained state and specified availability. |
| TC-008 | REQ-FUN-008 | Observe asynchronous changes, overflow retained history, and recover from a snapshot. | One contiguous event per revision; all transaction changes included; gaps return `410`; recovery restores current state. |
| TC-009 | REQ-PER-001 | Measure local releases; stall management; expire deadlines; adjust civil time; suspend or restart authority execution. | Timing bounds hold; civil time has no validity effect; discontinuity establishes a new boot and invalidates authority. |
| TC-010 | REQ-PER-002 | Execute the Level 2 baseline qualification run. | Required cycle count has zero missed handoffs; specified read-processing latency bound holds. |
| TC-011 | REQ-PER-003 | Execute Level 3 baseline and observation-overload runs. | Required cycle counts and control bounds hold; excess load receives explicit rejection when necessary. |
| TC-012 | REQ-SAF-001 | Review commissioning evidence and disconnect management during controlled motion. | Independent protection and local watchdog remain effective within the assessed profile. |
| TC-013 | REQ-SAF-002 | Request stopping; replay cached zero-velocity samples; delay fencing acknowledgment; exceed stop timeout. | Cached evidence cannot establish `STOPPED`; timeout latches and escalates; delayed commands cannot execute. |
| TC-014 | REQ-SAF-003 | Expire commands and leases separately and together; age feedback; fail critical health; assert protection. | Correct status and fault codes appear; stopping begins within bounds; no further ordinary output is authorized. |
| TC-015 | REQ-SAF-004 | Approach both bounded-joint guards; test inward recovery and out-of-range feedback. | Directional guards and fault rules hold without silent target clipping. |
| TC-016 | REQ-SEC-001 | Exercise each role against all endpoints and try foreign lease use. | Only authorized actions succeed; transport and identity checks hold. |
| TC-017 | REQ-SEC-002 | Fuzz structure and encodings; send oversized bodies; exhaust declared capacity. | Defined errors, bounded resource use, preserved deduplication obligations, and unaffected stop enforcement. |
| TC-018 | REQ-SEC-003 | Verify records; modify and remove retained records; interrupt audit persistence. | Tampering is detected against the trust anchor; new motion is rejected and stopping occurs on audit failure. |
| TC-019 | REQ-SEC-004 | Inspect stored and exported data; execute retention policy. | No raw credentials; access and retention match the declared policy and operational minimums. |
| TC-020 | REQ-INT-001 | Exchange maximum-range counters, malformed numbers, unsupported versions, and unknown members. | Exact encoding and version rules hold; unsupported data is rejected. |
| TC-021 | REQ-INT-002 | Apply known translations and rotations and compare joint-unit interpretation. | Transform direction, handedness, quaternion order, and SI units agree. |
| TC-022 | REQ-INT-003 | Reorder samples, skip sequences, inject future timestamps, and omit joints or dynamic frames. | Invalid samples are rejected or marked unavailable; freshness and completeness guards remain effective. |
| TC-023 | REQ-ACC-001 | Inspect included operator flows using keyboard and accessible text presentation. | Required state distinctions, units, labels, and operable actions are available. |
| TC-024 | REQ-RES-001 | Terminate authority execution during motion and restart while delayed device traffic remains. | New boot, old-command fencing, no automatic motion restoration, and uncertain-outcome reporting. |
| TC-025 | REQ-GOV-001 | Review assessment package and reproduce selected results from recorded configuration. | Claim, evidence, scope, limitations, and validity are traceable and internally consistent. |

Each applicable test MUST pass. A conditional test may be “not applicable” only when its triggering feature is absent from the assessed boundary.

### 8.3 Conformance report format

A conformance report MUST be a JSON object with these fields:

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `standard_id` | string | — | Yes | `WIA-ROB-020` | Assessed standard. |
| `spec_version` | string | — | Yes | `1.0` | Assessed version. |
| `report_id` | Name | — | Yes | Unique for assessor | Report identity. |
| `conformance_level` | integer enum | — | Yes | 1, 2, or 3 | Claimed level. |
| `assessor_id` | Name | — | Yes | Identified organization or team | Responsible assessor. |
| `build_identity` | Text | — | Yes | Reproducible release or artifact identity | Software under test. |
| `profile` | object | — | Yes | Fields below | Qualification conditions. |
| `results` | array | — | Yes | Every applicable test | Test outcomes. |
| `limitations` | Text array | — | Yes | Empty only if none identified | Boundaries and exclusions. |

`profile` MUST contain `hardware`, `operating_environment`, `driver_firmware`, `protective_mechanism`, `stop_procedure`, `stop_margin_evidence`, `resource_limits`, `retention_policy`, and `measurement_method`, each as `Text`, plus the tested `configuration: Configuration`.

Each result contains `test_id: string`, `requirement_ids: string[]`, `outcome`, `evidence: Text[]`, and `notes: Text`.

The complete outcome enumeration is `PASS`, `FAIL`, and `NOT_APPLICABLE`. The last value requires a reason.

Evidence MUST include measured timing distributions or raw timing records sufficient to verify extrema, injected-fault observations, relevant API traces, and identified physical commissioning evidence. An empty evidence array is not valid for `PASS`.

### 8.4 Audit record format

Required audit records contain:

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `boot_id` | Id128 | — | Yes | Source incarnation | Clock and authority domain. |
| `sequence` | Counter | — | Yes | Increasing within audit stream | Record order. |
| `monotonic_ns` | Time | ns | Yes | Event time | Local occurrence time. |
| `principal_id` | Name or null | — | Yes | Null for internal origin | Responsible actor. |
| `request_id` | Id128 or null | — | Yes | Null when no request exists | Request correlation. |
| `action` | Text | — | Yes | Exact method and path, or internal cause description | Recorded operation. |
| `outcome` | enum | — | Yes | Audit outcome | Disposition. |
| `revision` | Counter | — | Yes | Resulting or observed revision | Control-state correlation. |
| `detail` | Text | — | Yes | No credentials or unnecessary personal data | Relevant outcome information. |

The integrity storage mechanism and export packaging are implementation-defined and MUST be documented. Exported records MUST retain a verifiable relationship to their authenticated integrity evidence.

## 9. Certification, governance and versioning

Conformance assessment proceeds through boundary declaration, configuration review, commissioning-evidence review, applicable testing, report review, and issuance of an assessor’s statement.

A statement MUST identify the implementation, standard version, level, qualification profile, report identity, assessor, issue date, and expiry date. It MUST identify whether assessment was performed by the supplier or an independent assessor.

This specification does not establish a WIA accreditation organization, authorized certification mark, or official registry. An assessor’s statement MUST NOT imply an endorsement or functional-safety approval that has not separately been granted.

Assessment validity MUST end no later than 365 days after issuance. This interval is a review requirement of this standard, intended to force periodic confirmation of software, firmware, configuration, and commissioning assumptions.

Earlier reassessment is required after a material change, including:

- Changes to authority, scheduling, watchdog, fencing, or stop implementation.
- Changes to control-period or timeout values outside the assessed configuration.
- Changes to driver firmware, actuator interfaces, or protective mechanisms.
- Changes to load or operating conditions invalidating stopping evidence.
- Changes to authentication, audit integrity, or resource isolation.
- Correction of a defect that affected a previously claimed requirement.

An unchanged requirement may reuse prior evidence when the assessor documents why the change cannot affect it. Affected requirements MUST be retested.

The steward of a published revision MUST maintain an issue record, technical rationale, compatibility assessment, and approved revision text. Conflicting normative rules require an explicit correction; implementations MUST NOT silently select a preferred interpretation and claim unqualified conformance.

Editorial corrections may clarify wording without changing accepted data or behavior. Additive negotiated capabilities may receive a minor version. Changes to existing semantics require a major version.

A future revision MUST NOT reinterpret an existing ENUM value, change a field’s unit, or weaken an existing authority or stop guarantee while retaining the same negotiated version. Historical conformance reports remain claims about their stated version and profile.

## Appendix A: Example payloads

All payloads and operating values in this appendix are illustrative. They do not document an actual robot installation.

### A.1 Lease acquisition

The example request uses a previously observed runtime revision and monotonic clock value.

```json
{
  "boot_id": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
  "request_id": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
  "expected_revision": "6",
  "expires_at_ns": "1500000000"
}
```

Illustrative response for acceptance at boot time 1,000,000,000 ns with the example configuration’s lease duration:

```json
{
  "boot_id": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
  "request_id": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
  "revision": "7",
  "accepted_at_ns": "1000000000",
  "replayed": false,
  "result": {
    "lease_id": "cccccccccccccccccccccccccccccccc",
    "owner_id": "operator-demo",
    "epoch": "1",
    "granted_at_ns": "1000000000",
    "expires_at_ns": "2000000000"
  }
}
```

### A.2 Complete velocity command request

For this example, the runtime remains at revision `7`, the joint is within its directional guard, and acceptance occurs at 1,050,000,000 ns. The requested deadline leaves the example configuration’s full 100,000,000 ns command-validity interval.

```json
{
  "boot_id": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
  "request_id": "dddddddddddddddddddddddddddddddd",
  "expected_revision": "7",
  "expires_at_ns": "1150000000",
  "configuration_id": "cfg-slide-1",
  "lease_id": "cccccccccccccccccccccccccccccccc",
  "epoch": "1",
  "mode": "JOINT_VELOCITY",
  "targets": [
    {
      "joint_id": "slide",
      "velocity": 0.1
    }
  ]
}
```

Illustrative acceptance response:

```json
{
  "boot_id": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
  "request_id": "dddddddddddddddddddddddddddddddd",
  "revision": "8",
  "accepted_at_ns": "1050000000",
  "replayed": false,
  "result": {
    "command_id": "dddddddddddddddddddddddddddddddd",
    "configuration_id": "cfg-slide-1",
    "lease_id": "cccccccccccccccccccccccccccccccc",
    "epoch": "1",
    "mode": "JOINT_VELOCITY",
    "status": "ACCEPTED",
    "accepted_at_ns": "1050000000",
    "updated_at_ns": "1050000000",
    "expires_at_ns": "1150000000",
    "targets": [
      {
        "joint_id": "slide",
        "velocity": 0.1
      }
    ]
  }
}
```

This response establishes acceptance only. The first successful handoff creates another revision and changes status to `EXECUTING`.

### A.3 Joint-state sample

This illustrative sample reports metres, metres per second, and newtons because `slide` is a prismatic joint.

```json
{
  "spec_version": "1.0",
  "runtime_id": "runtime-demo",
  "boot_id": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
  "topic": "joint_state",
  "sequence": "106",
  "monotonic_ns": "1060000000",
  "payload": {
    "joints": [
      {
        "joint_id": "slide",
        "position": 0.50005,
        "velocity": 0.005,
        "effort": null
      }
    ]
  }
}
```

The measured velocity need not equal the requested target because acceleration limiting and physical response occur after acceptance.

### A.4 Deactivation request

This illustrative request assumes the client has just observed revision `12`.

```json
{
  "boot_id": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
  "request_id": "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",
  "expected_revision": "12",
  "expires_at_ns": "1800000000",
  "action": "DEACTIVATE"
}
```

Acceptance enters `STOPPING`, revokes the lease, advances the epoch, and cancels a current command. A later observation must establish `STOPPED` and `INACTIVE`; the request itself does not establish either condition.

### A.5 Fault snapshot

The following independent illustrative snapshot shows command expiry followed by an unconfirmed stop. The lifecycle and physical stop status intentionally communicate different facts.

```json
{
  "spec_version": "1.0",
  "runtime_id": "runtime-demo",
  "robot_id": "robot-demo",
  "boot_id": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
  "revision": "15",
  "observed_at_ns": "2200000000",
  "state": "FAULT",
  "stop_status": "UNKNOWN",
  "protective_state": "CLEAR",
  "motion_permitted": false,
  "configuration_id": "cfg-slide-1",
  "epoch": "2",
  "lease": null,
  "current_command": null,
  "fault": {
    "codes": [
      "WATCHDOG_EXPIRED",
      "STOP_TIMEOUT"
    ],
    "detected_at_ns": "1150000000",
    "detail": "The command expired. Fresh stop-confirmation evidence was not obtained before the configured stop deadline."
  }
}
```

## Appendix B: Revision history

v1.0 — initial release