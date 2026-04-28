# WIA-DEF-001 — Phase 2: API Interface

> Unmanned-weapon canonical Phase 2: API surface (platforms + missions + engagements + audit + telemetry + C2).

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


## 4. Targeting Systems

### 4.1 Target Detection

#### 4.1.1 Sensor Fusion

Combine multiple sensor inputs:
```
Confidence = w₁·Radar + w₂·EO + w₃·IR + w₄·Acoustic
```

Where:
- `w₁, w₂, w₃, w₄` = Sensor weights (sum to 1)
- Each sensor contributes confidence score (0-1)

**Minimum confidence for target lock: 0.90**

#### 4.1.2 Machine Learning Classification

Neural network-based target identification:
```
Classification = CNN(sensor_data)
```

**Requirements**:
- Training dataset: ≥100,000 labeled examples
- Validation accuracy: ≥98%
- False positive rate: <0.5%
- Model retraining: Monthly or after significant misclassification

#### 4.1.3 IFF Integration

Identification Friend or Foe mandatory:
```
if IFF_Response == "FRIENDLY":
    abort_engagement()
    log_near_miss()
elif IFF_Response == "NEUTRAL":
    escalate_to_human()
elif IFF_Response == "HOSTILE":
    proceed_engagement_protocol()
else:  # No response
    escalate_to_human()
```

### 4.2 Threat Assessment

#### 4.2.1 Threat Scoring Algorithm

```
Threat_Score = (
    α · Capability_Score +
    β · Intent_Score +
    γ · Proximity_Score +
    δ · Velocity_Score
)
```

Where:
- `Capability_Score` = Estimated destructive potential (0-1)
- `Intent_Score` = Assessed hostile intent (0-1)
- `Proximity_Score` = Distance to protected asset (0-1, inverse)
- `Velocity_Score` = Speed toward asset (0-1)
- `α + β + γ + δ = 1`

**Default weights**: α=0.3, β=0.35, γ=0.2, δ=0.15

**Threat classification**:
- `Threat_Score < 0.3`: Low threat, monitor
- `0.3 ≤ Threat_Score < 0.6`: Medium threat, track and alert
- `0.6 ≤ Threat_Score < 0.8`: High threat, prepare engagement
- `Threat_Score ≥ 0.8`: Critical threat, engage if authorized

#### 4.2.2 Proportionality Assessment

```
Proportionality = min(
    Threat_Level,
    Response_Intensity
) / max(
    Collateral_Risk,
    0.01  # Prevent division by zero
)
```

**Minimum proportionality score for engagement: 8.0**

### 4.3 Engagement Calculation

#### 4.3.1 Engagement Authorization Score

```
Engagement_Score = (
    Threat_Level ×
    Target_Certainty ×
    Proportionality ×
    IFF_Clear ×
    ROE_Compliance
)
```

Where each factor is 0-1 (except IFF_Clear which is binary 0 or 1).

**Minimum score for autonomous engagement**:
- Level 2: N/A (human required)
- Level 3: 0.95
- Level 4: 0.98

#### 4.3.2 Optimal Engagement Timing

```
T_optimal = argmin(
    Time_to_Impact - Weapon_Flight_Time - Decision_Time
)
```

Subject to:
```
Probability_of_Kill(T_optimal) ≥ 0.85
Collateral_Damage(T_optimal) ≤ 0.05
```

#### 4.3.3 Weapon Selection

For multiple payload options:
```
Weapon = argmax(
    Effectiveness(weapon, target) ×
    Availability(weapon) ×
    (1 - Collateral_Risk(weapon))
)
```

---



## 9. Implementation Guidelines

### 9.1 Required Components

Any WIA-DEF-001 compliant system must include:

1. **Weapon Controller**: Hardware/software for weapon management
2. **Targeting System**: Sensor fusion and target acquisition
3. **IFF System**: Friend-or-foe identification
4. **Safety Monitor**: Real-time safety checks
5. **Ethical Module**: Compliance verification
6. **Communications**: Secure C2 links
7. **Logging System**: Complete audit trail
8. **Geofencing**: Operational boundary enforcement

### 9.2 API Interface

#### 9.2.1 Weapon Configuration

```typescript
interface WeaponConfig {
  type: 'UGV' | 'UAV' | 'USV' | 'UUV' | 'Sentry' | 'Loitering';
  autonomyLevel: 0 | 1 | 2 | 3 | 4;
  payload: PayloadConfig;
  sensors: SensorConfig[];
  range: number;  // meters
  maxSpeed: number;  // m/s
  endurance: number;  // seconds
}
```

#### 9.2.2 Targeting Request

```typescript
interface TargetingRequest {
  targetId: string;
  position: Coordinate3D;
  velocity: Vector3D;
  classification: TargetClass;
  threatLevel: number;  // 0-1
  certainty: number;  // 0-1
  iffStatus: 'friendly' | 'neutral' | 'hostile' | 'unknown';
}
```

#### 9.2.3 Engagement Authorization

```typescript
interface EngagementAuth {
  authorized: boolean;
  reasoning: string[];
  engagementScore: number;
  humanApprovalRequired: boolean;
  weaponSelected: string;
  estimatedCollateralDamage: number;
  ethicalCompliance: EthicalCheck;
}
```

### 9.3 Data Formats

#### 9.3.1 Geofence Definition

```json
{
  "geofence_id": "GF-2025-001",
  "type": "polygon",
  "coordinates": [
    {"lat": 37.5, "lon": 127.0, "alt": 0},
    {"lat": 37.6, "lon": 127.1, "alt": 5000},
    ...
  ],
  "valid_from": "2025-01-01T00:00:00Z",
  "valid_until": "2025-12-31T23:59:59Z"
}
```

#### 9.3.2 Rules of Engagement (ROE)

```json
{
  "roe_id": "ROE-2025-DEF-001",
  "autonomy_authorized": 2,
  "weapons_free_zones": ["GF-2025-001"],
  "weapons_hold_zones": ["GF-2025-002"],
  "no_fire_zones": ["GF-2025-003"],
  "authorized_targets": ["hostile-uav", "hostile-missile"],
  "prohibited_targets": ["civilian", "medical", "cultural"],
  "max_collateral_damage": 0.05,
  "escalation_of_force_required": true
}
```

### 9.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| D001 | Target identification failure | Abort engagement, escalate to human |
| D002 | IFF system malfunction | Weapons safe, return to base |
| D003 | Geofence violation | Correct course, alert operator |
| D004 | Communication loss | Execute lost-link procedure |
| D005 | Ethical violation detected | Abort, log incident, investigate |
| D006 | Sensor malfunction | Reduce autonomy level, notify operator |
| D007 | Weapon malfunction | Safe weapon, abort mission |
| D008 | Swarm coherence lost | Reform sub-swarms, reduce scope |

---




---

## A.1 Endpoint reference

```http
POST /unmanned-weapon/v1/platforms                # register platform record
GET  /unmanned-weapon/v1/platforms/{id}           # fetch platform record
POST /unmanned-weapon/v1/missions                 # plan a mission
GET  /unmanned-weapon/v1/missions/{id}/state      # current mission state
POST /unmanned-weapon/v1/engagements              # record engagement event (privileged)
GET  /unmanned-weapon/v1/engagements/{id}/audit   # full audit trail
WS   /unmanned-weapon/v1/state/stream             # platform state telemetry
WS   /unmanned-weapon/v1/c2/control               # command-and-control link
```

Every endpoint follows the discovery convention at `/.well-known/wia-unmanned-weapon`. Engagement-recording endpoints require a chain-of-command credential at WIA-OMNI-API trust tier 3 plus a fresh-quorum signature.

## A.2 Platform-record API

`POST /platforms` accepts the Phase 1 §A.1 envelope and returns a stable `platformId`. Subsequent characterisation reports (acceptance-test, periodic-recurrent-airworthiness, post-incident inspection) reference the `platformId`. Updates to the canonical record produce a new version with the prior version preserved in the version history. Read endpoints expose the per-platform configuration history so operators can verify that the deployed configuration matches the airworthiness authorisation.

## A.3 Mission-planning API

`POST /missions` accepts a mission-plan envelope: platform reference, autonomy level, route waypoints with altitude-and-speed envelope, the engagement-rules envelope reference per Phase 1 §A.4, the geo-fence envelope, the launch-and-recovery procedures, the payload-arming envelope (where applicable), the abort-channel reference, and the chain-of-command sign-off. Mission plans are gated by the operating authority's release process: a plan can be in states `draft`, `under-review`, `approved`, `armed`, `executing`, `completed`, `aborted`, with state transitions emitted as audit events.

## A.4 Engagement and audit API

`POST /engagements` registers an engagement event; the request is gated by a chain-of-command credential and requires a fresh-quorum signature plus a signed targeting-confidence packet. `GET /engagements/{id}/audit` returns the immutable audit trail for an engagement, including the chain-of-command authorisation, the targeting-confidence trace, the human-on-the-loop or human-in-the-loop confirmation event, the platform-state at the engagement instant, and the post-engagement assessment. Audit-trail integrity is anchored into a Merkle tree per-tenant; chain breaks invalidate the engagement record and trigger a forensic-review event.

## A.5 Telemetry and C2 WebSocket

`/state/stream` multiplexes platform telemetry (position, velocity, battery, link RSSI, sensor status, payload status), the alarm channel (loss-of-link, loss-of-GPS, kill-switch armed, geo-fence breach), and the mission-progress events. `/c2/control` carries the bi-directional command-and-control link with mTLS plus the operator's credential and the per-message signature; control messages are rate-limited per the platform's command-bandwidth envelope and replay-protected per Phase 3 §A.6.

## A.6 Rate-limit and access-control envelope

Read endpoints: 1000 req/h authenticated, 5000 req/h trusted-partner. Write endpoints: gated per credential class (maintenance crew, mission planner, chain-of-command) with the operator's rate-limit policy applied. Audit-trail reads require the operator's investigative-credential or a court-order envelope; export of audit trails outside the operator's tenant boundary is disallowed without a documented bilateral agreement and the appropriate data-residency tags.

## A.7 Webhook delivery for state transitions

Operators can subscribe to webhook deliveries on mission state transitions and on engagement events. Webhook payloads carry the same envelope shape as the API GET response, signed by the WIA tenant key, and retried with exponential backoff up to 24 h before the broker logs a permanent-failure event for operator review. Subscriber endpoints MUST validate the webhook signature against the tenant key chain before processing; failure to validate is a conformance defect and is reported to the WIA monitoring sink.


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
