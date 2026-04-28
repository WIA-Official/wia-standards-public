# WIA-DEF-001 — Phase 1: Data Format

> Unmanned-weapon canonical Phase 1: platform + autonomy-level + targeting + engagement-rules + safety envelopes.

# WIA-DEF-001: Unmanned Weapon Specification v1.0

> **Standard ID:** WIA-DEF-001
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Defense & Security Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Weapon Classification](#2-weapon-classification)
3. [Autonomy Levels](#3-autonomy-levels)
4. [Targeting Systems](#4-targeting-systems)
5. [Engagement Rules](#5-engagement-rules)
6. [Swarm Coordination](#6-swarm-coordination)
7. [Safety Protocols](#7-safety-protocols)
8. [Ethical Framework](#8-ethical-framework)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---


## 2. Weapon Classification

### 2.1 Platform Types

#### 2.1.1 Unmanned Aerial Vehicles (UAV)

Fixed-wing and rotary unmanned aircraft for:
- Air defense interception
- Intelligence, Surveillance, Reconnaissance (ISR)
- Precision strike (defensive only)
- Electronic warfare

**Specifications**:
```
Altitude Range: 0-15,000 meters
Speed Range: 10-300 m/s
Endurance: 1-48 hours
Payload: 0-200 kg
```

#### 2.1.2 Unmanned Ground Vehicles (UGV)

Tracked or wheeled platforms for:
- Perimeter security
- Explosive ordnance disposal (EOD)
- Convoy protection
- Urban defense

**Specifications**:
```
Terrain: Urban, Desert, Forest, Mountain
Speed: 0-20 m/s
Endurance: 4-72 hours
Payload: 10-500 kg
```

#### 2.1.3 Unmanned Surface Vehicles (USV)

Maritime surface platforms for:
- Port security
- Anti-piracy operations
- Mine countermeasures
- Vessel interdiction

**Specifications**:
```
Displacement: 50-5,000 kg
Speed: 5-50 knots
Endurance: 8-120 hours
Range: 50-500 km
```

#### 2.1.4 Unmanned Underwater Vehicles (UUV)

Submarine platforms for:
- Harbor defense
- Anti-submarine warfare
- Mine detection/clearance
- Infrastructure protection

**Specifications**:
```
Depth: 0-1000 meters
Speed: 2-15 knots
Endurance: 10-200 hours
Range: 20-300 km
```

#### 2.1.5 Sentry Systems

Static or mobile defensive emplacements:
- Perimeter defense
- Critical infrastructure protection
- Counter-UAS (Unmanned Aerial System)
- Area denial

**Specifications**:
```
Coverage: 360° or directional
Range: 100-5,000 meters
Detection: Radar, EO/IR, Acoustic
Response: Kinetic, Electronic, Directed Energy
```

#### 2.1.6 Loitering Munitions

Expendable systems that:
- Patrol designated areas
- Identify and engage threats
- Self-destruct if not employed
- Provide ISR capability

**Specifications**:
```
Loiter Time: 15-120 minutes
Range: 5-100 km
Payload: 1-40 kg
Guidance: GPS, EO, IR, RF homing
```

### 2.2 Payload Classification

#### 2.2.1 Kinetic Weapons
- Precision-guided missiles
- Small-caliber weapons (5.56-30mm)
- Medium-caliber weapons (30-76mm)
- Anti-armor weapons

#### 2.2.2 Non-Lethal Weapons
- Rubber bullets / beanbag rounds
- Tear gas / smoke dispensers
- Acoustic deterrents
- Laser dazzlers

#### 2.2.3 Electronic Countermeasures
- Jamming systems
- Signal interceptors
- Cyber warfare tools
- GPS/communications denial

#### 2.2.4 Directed Energy Weapons
- High-power microwave (HPM)
- Laser systems
- Electromagnetic pulse (EMP)

---



## 3. Autonomy Levels

### 3.1 Level 0: Manual Control

**Description**: Human operator controls all functions remotely.

**Requirements**:
- Real-time video feed required
- Latency < 200ms for control
- Operator maintains line-of-sight or reliable communications
- No autonomous decision-making

**Use Cases**:
- Explosive ordnance disposal
- Precision operations
- Training exercises

### 3.2 Level 1: Assisted Control

**Description**: System provides recommendations; human makes all decisions.

**Requirements**:
- AI-assisted target detection
- Threat assessment displays
- Recommended actions presented
- Human approval required for all actions
- Override capability always available

**Use Cases**:
- Complex threat environments
- ISR with engagement option
- Training autonomous systems

### 3.3 Level 2: Semi-Autonomous

**Description**: System can identify and track targets; human approval required for engagement.

**Requirements**:
- Automated target acquisition
- IFF integration mandatory
- Human authorization required before firing
- Maximum 30-second decision window
- Auto-abort if no human confirmation

**Use Cases**:
- Air defense against fast-moving threats
- Perimeter security with human oversight
- Counter-UAS operations

**Minimum Engagement Criteria**:
```
Target_Certainty ≥ 0.95
Threat_Level ≥ 0.8
IFF_Confirmation = True
Human_Authorization = True
```

### 3.4 Level 3: Conditional Autonomy

**Description**: System operates autonomously within defined parameters; human can override.

**Requirements**:
- Predefined engagement zone (geofenced)
- Specific threat profiles authorized
- Human supervisor can veto any action
- Real-time telemetry to command center
- Automatic reporting of all engagements

**Use Cases**:
- Critical infrastructure protection
- Anti-missile defense
- Swarm defense operations

**Constraints**:
```
Geofence: Strictly enforced
Time_Window: Maximum 4 hours autonomous
Threat_Types: Predefined list only
Collateral_Risk: < 0.01 (1%)
```

### 3.5 Level 4: High Autonomy

**Description**: System operates independently with human supervision only.

**Requirements**:
- Advanced AI decision-making
- Multi-sensor fusion and verification
- Ethical decision framework integrated
- Human can abort at any time
- Post-engagement reporting mandatory

**Use Cases**:
- Missile interception (time-critical)
- Swarm coordination
- Area denial operations

**Special Authorization Required**:
- Theater commander approval
- Legal review completed
- ROE explicitly permit Level 4
- Emergency defensive scenarios only

**Prohibition**: Level 4 autonomous lethal action against humans prohibited except when:
1. Imminent threat to human life
2. No time for human authorization (< 5 seconds)
3. ROE explicitly authorize defensive response
4. Post-action review guaranteed

---




---

## A.1 Platform-record envelope

The Phase 1 envelope groups platform records by domain (UAV — fixed-wing, rotary, hybrid VTOL; UGV — wheeled, tracked, legged; USV — surface, semi-submersible; UUV — torpedo-form, glider, AUV) with the canonical fields: platform identifier, platform class per the operator nation's defence registry, mass empty in kg, mass takeoff/payload in kg, dimensions (length, span, height) in metres, propulsion (electric — battery chemistry/energy density; combustion — fuel type, specific consumption; hybrid), endurance in hours, dash-speed in km/h, payload-bay envelope (mass, volume, electrical bus), the operating-environment envelope (sea-state, altitude band, temperature/humidity, EMI), and the manufacturer-of-record certificate chain. Platform data follows the reporting conventions of the relevant national airworthiness authority (FAA Part 107 / ASTM F38 / EASA SC-VTOL / KOCA UAS) plus the cross-domain ISO 21384 (UAS) and IEC 63402 (UGV functional safety).

## A.2 Autonomy-level descriptor

The autonomy-level descriptor follows the SAE J3016-style decomposition adapted for unmanned-weapon platforms: Level 0 fully teleoperated; Level 1 assisted (operator-in-the-loop with autonomous stabilisation); Level 2 supervised (semi-autonomous waypoint and return-to-base, operator approves engagement); Level 3 conditional (autonomous mission segments under operator authority); Level 4 high (autonomous within a defined geographic and rules-of-engagement envelope, operator can override); Level 5 reserved for research-only configurations and is NOT certified for engagement. Each descriptor carries the rules-of-engagement reference, the human-on-the-loop / human-in-the-loop / human-out-of-the-loop classification, and the kill-switch envelope.

## A.3 Targeting-system descriptor

A targeting descriptor MUST list the sensor stack (EO/IR camera with focal length and FOV; laser rangefinder with class per IEC 60825-1 and maximum range; radar with band, peak power, and antenna gain; SAR with resolution and swath; SIGINT receiver with frequency coverage), the target-classification envelope (combatant, vehicle-type, asset-type categorisation per the operator's published taxonomy), the confidence threshold for human review, and the false-positive / false-negative envelope per the operator's acceptance test. Targeting-system descriptors cross-reference the engagement-rules envelope at Phase 3 §A.1 so the system cannot fire a target the rules do not authorise.

## A.4 Engagement-rules envelope

Engagement-rules envelopes carry: rules-of-engagement (ROE) version identifier issued by the chain of command, the per-target-class authorisation matrix, the geographic-fence envelope (geo-fence polygons in WGS 84 with admin-1 boundaries), the time-of-day window, the de-escalation matrix (challenge / warning / disabling-fire / lethal-force), the proportionality envelope (collateral-damage estimate per IHL Article 51 (5)(b) Additional Protocol I), and the abort-channel reference. Envelopes are signed by the chain-of-command's key chain and validated at engagement-decision time.

## A.5 Safety descriptor

Safety descriptors carry: kill-switch redundancy class (single-channel, dual-channel, triple-channel), failure-mode envelope (loss-of-link → orbit and recover; loss-of-GPS → INS dead-reckoning bounded; battery low → return-to-base; hostile-spoof → safe-mode and report), the SOTIF (ISO 21448) envelope where applicable, the cyber-resilience envelope (signed firmware, secure-boot chain, hardware-anchored attestation), and the after-action-review log retention envelope (typically 7 years for engagement events, 1 year for non-engagement events). Descriptors include the explicit-non-engagement-state declaration (the platform's "safe state" used during anomaly handling).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/unmanned-weapon/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-unmanned-weapon-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/unmanned-weapon-host:1.0.0` ships every unmanned-weapon envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/unmanned-weapon.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Unmanned-weapon deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
