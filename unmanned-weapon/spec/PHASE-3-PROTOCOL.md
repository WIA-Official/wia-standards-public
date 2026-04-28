# WIA-DEF-001 — Phase 3: Protocol

> Unmanned-weapon canonical Phase 3: protocols (engagement-rules + swarm + safety + cybersecurity + de-escalation).

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


## 5. Engagement Rules

### 5.1 Rules of Engagement (ROE) Framework

#### 5.1.1 Defensive ROE Hierarchy

1. **Self-Defense**: Protect the unmanned system itself
2. **Unit Defense**: Protect other friendly units
3. **Asset Defense**: Protect designated critical assets
4. **Mission Defense**: Protect mission objectives
5. **Civilian Protection**: Minimize civilian harm (always)

#### 5.1.2 Escalation of Force

Progressive response ladder:
```
1. Warning (visual, audio, radio)
2. Deterrence (show of force, maneuvering)
3. Non-lethal engagement (EW, dazzlers, nets)
4. Disabling fire (immobilize, not destroy)
5. Lethal engagement (only if no alternative)
```

**Requirement**: Attempt lower levels before escalation unless threat is imminent.

### 5.2 Positive Identification Requirements

Before engagement authorization:
```
1. Target detected by ≥2 independent sensors
2. Target classified with ≥95% confidence
3. IFF interrogation negative or non-responsive
4. Visual confirmation (if possible)
5. Target behavior consistent with hostile intent
6. No friendly forces in engagement zone
7. Collateral damage assessment acceptable
```

### 5.3 No-Fire Zones

#### 5.3.1 Absolute No-Fire Zones

Engagement strictly prohibited:
- Hospitals and medical facilities
- Schools and universities
- Places of worship
- Civilian refugee camps
- Cultural heritage sites
- Explicitly designated safe zones

#### 5.3.2 Conditional No-Fire Zones

Engagement requires elevated authorization:
- Urban areas (civilian population)
- Dual-use infrastructure
- Border regions
- International waters/airspace

### 5.4 Collateral Damage Estimation

#### 5.4.1 Civilian Casualty Estimation (CCE)

```
CCE = Σ(P_civilian_i × P_casualty_i)
```

Where:
- `P_civilian_i` = Probability of civilian presence at location i
- `P_casualty_i` = Probability of casualty at location i given engagement

**Maximum acceptable CCE**: 0.05 (5% probability of civilian casualty)

**Special restrictions**:
- CCE > 0.01: Human approval required
- CCE > 0.05: Engagement prohibited except imminent threat to life
- CCE > 0.10: Engagement prohibited absolutely

#### 5.4.2 Infrastructure Damage Assessment

```
IDA = Σ(Value_i × Damage_Probability_i)
```

**Acceptable thresholds**:
- Critical infrastructure: IDA < 0.10
- Civilian infrastructure: IDA < 0.20
- Dual-use: IDA < 0.30

---



## 6. Swarm Coordination

### 6.1 Swarm Architecture

#### 6.1.1 Hierarchical Organization

```
Swarm Leader (Level 3-4 autonomy)
├── Sub-swarm Alpha (3-5 units)
├── Sub-swarm Bravo (3-5 units)
└── Sub-swarm Charlie (3-5 units)
```

**Communication protocol**:
- Leader ↔ Human: Secure encrypted link
- Leader ↔ Sub-leaders: High-bandwidth tactical data
- Swarm members: Mesh network, peer-to-peer

#### 6.1.2 Swarm Size Limits

- **Minimum**: 3 units (for redundancy)
- **Optimal**: 5-12 units (coordination efficiency)
- **Maximum**: 50 units (control complexity limit)

### 6.2 Coordination Algorithms

#### 6.2.1 Distributed Task Allocation

Hungarian algorithm for optimal assignment:
```
Cost_Matrix[unit][task] = (
    Distance(unit, task) +
    Capability_Match(unit, task) +
    Fuel_Constraint(unit)
)

Assignment = Hungarian_Method(Cost_Matrix)
```

#### 6.2.2 Formation Control

Maintain tactical formation using potential fields:
```
F_i = Σ F_attraction(target) + Σ F_repulsion(other_units) + F_obstacle_avoidance

v_i = K_p × F_i + K_d × (v_desired - v_current)
```

**Standard formations**:
- **Line Abreast**: Wide area coverage
- **Wedge**: Frontal assault/defense
- **Diamond**: 360° awareness
- **Column**: Transit through narrow spaces
- **Swarm**: Overwhelming concentration

#### 6.2.3 Consensus-Based Decision Making

Distributed voting protocol:
```
1. Each unit assesses situation locally
2. Units share assessments via mesh network
3. Consensus reached if ≥75% agree
4. If no consensus, escalate to human or swarm leader
```

### 6.3 Swarm Tactics

#### 6.3.1 Saturation Attack Defense

Overwhelm enemy defenses with simultaneous multi-vector approach:
```
Time_on_Target: ±500ms synchronization
Approach_Vectors: ≥3 distinct directions
Decoy_Ratio: 2:1 (decoys to actual threats)
```

#### 6.3.2 Area Denial

Establish persistent defensive perimeter:
```
Coverage = Union(Coverage_i for all units)
Overlap_Ratio ≥ 1.5 (no gaps)
Patrol_Pattern: Randomized to prevent predictability
```

#### 6.3.3 Adaptive Response

Swarm adapts to enemy tactics:
```
if Enemy_Tactic == "Jamming":
    switch_to_autonomous_mode()
    increase_unit_spacing()
elif Enemy_Tactic == "Area_Weapons":
    disperse_formation()
    increase_altitude/depth
elif Enemy_Tactic == "Cyber_Attack":
    isolate_compromised_units()
    revert_to_manual_control()
```

### 6.4 Fail-Safe in Swarm Operations

#### 6.4.1 Loss of Communication

```
if communication_lost(timeout=30s):
    execute_predetermined_safe_action()
    # Options:
    # - Return to base
    # - Loiter at current position
    # - Continue mission (only if Level 4 authorized)
    # - Self-destruct (if in hostile territory and sensitive)
```

#### 6.4.2 Swarm Disaggregation

If swarm coherence lost:
```
if swarm_coherence < 0.6:
    reform_smaller_sub_swarms()
    reassign_tasks()
    notify_human_operator()
```

#### 6.4.3 Ethical Override in Swarm

Any single unit can veto swarm engagement if:
```
civilian_risk > threshold OR
IFF_uncertainty > 0.1 OR
proportionality < minimum
```

**Veto propagates to entire swarm immediately.**

---



## 7. Safety Protocols

### 7.1 Pre-Deployment Checks

#### 7.1.1 System Validation Checklist

- [ ] Hardware diagnostics passed
- [ ] Software version verified and approved
- [ ] Weapons safed and verified
- [ ] IFF system tested and operational
- [ ] Communications links established
- [ ] Geofence loaded and confirmed
- [ ] ROE uploaded and acknowledged
- [ ] Operator trained and certified
- [ ] Emergency abort tested

#### 7.1.2 Mission Authorization

Required approvals:
- **Level 0-1**: Tactical commander
- **Level 2**: Operational commander
- **Level 3**: Theater commander + legal review
- **Level 4**: National command authority + ethics board

### 7.2 Operational Safety Mechanisms

#### 7.2.1 Dead Man's Switch

```
if no_operator_input(timeout=300s):  # 5 minutes
    weapons_safe()
    return_to_base()
    alert_command()
```

#### 7.2.2 Geofence Enforcement

```
if position outside authorized_zone:
    weapons_safe()
    alter_course(toward=authorized_zone)
    alert_operator()

if unable_to_return(fuel/time):
    emergency_landing()
    self_destruct_payload(if_required)
```

#### 7.2.3 Friendly Fire Prevention

Multi-layered protection:
```
1. IFF transponder interrogation
2. Blue Force Tracker integration
3. Visual identification (if capable)
4. Operator override capability
5. Minimum separation distance enforcement

if any_friendly_indicator:
    abort_engagement()
    mark_as_friendly()
    log_incident()
```

#### 7.2.4 Malfunction Detection

```
if system_error_detected():
    weapons_safe()
    enter_safe_mode()
    notify_operator()

    if critical_failure:
        emergency_landing()
        disable_weapons_permanently()
```

### 7.3 Post-Engagement Protocol

#### 7.3.1 Battle Damage Assessment (BDA)

```
1. Confirm target neutralization
2. Assess collateral damage
3. Detect secondary threats
4. Document with sensor data
5. Report to command immediately
```

#### 7.3.2 Incident Reporting

Required for all engagements:
```json
{
  "timestamp": "ISO-8601",
  "system_id": "DEF-UAV-001",
  "target_id": "TGT-2025-001",
  "engagement_score": 0.97,
  "authorization": "Human/Autonomous",
  "outcome": "Success/Failure/Abort",
  "collateral_damage": 0.0,
  "sensor_data": "link_to_recordings",
  "review_status": "Pending"
}
```

#### 7.3.3 After-Action Review (AAR)

Mandatory review within 72 hours:
- Technical performance analysis
- Engagement decision review
- Ethical compliance verification
- Lessons learned documentation
- System updates if needed

---




---

## A.1 Engagement-rules protocol

The engagement-rules protocol covers the chain-of-authorisation handshake (mission-planner proposes → command authority approves → field commander executes), the per-target-class authorisation matrix, and the abort channel. Every engagement decision passes through a deterministic decision tree with leaves: deny / challenge / warn / disable / lethal. Each leaf carries the human-on-the-loop or human-in-the-loop confirmation requirement (Levels 0-3 require explicit operator confirmation; Level 4 may execute autonomously within a pre-authorised target class but emits a real-time notification to the operator with an abort window). Level 5 configurations are NOT certified for engagement.

## A.2 Swarm-coordination protocol

Swarm-coordination protocol covers the formation envelope (leader-follower, V-formation, distributed-decentralised, hierarchical clusters), the inter-platform link (mesh radio at 900 MHz / 2.4 GHz / 5 GHz / mmWave; LEO satellite for over-the-horizon; line-of-sight optical for stealthy ops), the consensus algorithm (Raft, PBFT-like for byzantine resilience), the de-confliction envelope (4D trajectory negotiation; deconfliction-window negotiation when corridor conflicts arise), and the swarm-level kill-switch (any platform can broadcast a swarm-abort that all peers must honour within the documented latency budget).

## A.3 Safety protocol

The safety protocol covers loss-of-link recovery (orbit at last commanded waypoint; return-to-base after pre-set delay; safe-landing if return-to-base impossible), loss-of-GPS recovery (INS dead-reckoning bounded by error growth; visual-INS terrain-relative navigation where supported; safe-mode if both fail), low-battery recovery (return-to-base if reachable; safe-landing in pre-authorised emergency zone otherwise), hostile-spoof recovery (signal anomaly detector → safe-mode → operator notification), and ground-collision-avoidance protocol per the platform's ASTM F3322 / ISO 21384-3 envelope.

## A.4 Cybersecurity protocol

The cybersecurity protocol covers the platform's secure-boot chain (hardware root-of-trust → bootloader → OS → applications, each step measured into the TPM-equivalent), the signed-firmware envelope (vendor-signed releases with the operator's counter-signature for fielded units), the C2 link protection (mTLS with chain-of-command-issued certificates; per-message Ed25519 signatures; replay-protection per Phase 3 §A.6), the over-the-air-update protocol (delta-update with rollback-protection; staged rollout to a single platform before fleet propagation), and the incident-response envelope (compromised platforms are quarantined and forensic-imaged; incident summaries are shared with the operator's CSIRT and with the WIA-AIR-SHIELD trust list).

## A.5 De-escalation and rules-of-engagement protocol

De-escalation follows the operator's published Rules of Engagement (ROE) and the controlling international humanitarian law (IHL) framework: distinction (Article 48 Additional Protocol I — combatants vs. civilians), proportionality (Article 51 (5)(b) — collateral-damage rule), and precaution (Article 57 — feasible precautions in attack). The protocol enforces challenge-warn-disable-lethal escalation by default; the chain of command can authorise direct-engagement only for the target classes the ROE permits and only within the engagement-rules envelope's geographic and time windows.

## A.6 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for the C2 control plane. Telemetry traffic uses mTLS with per-channel monotonic counters; replay attempts are detected and dropped at the broker. Engagement-decision audit logs are hash-chained per platform with the chain anchored into a Merkle tree per-operator so revisions to the decision history can be detected during post-engagement review. The integrity-defence envelope enumerates the acceptable cryptographic suite per IETF RFC 9325 (TLS recommendations) plus the post-quantum migration path per NIST FIPS 203/204/205.


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
