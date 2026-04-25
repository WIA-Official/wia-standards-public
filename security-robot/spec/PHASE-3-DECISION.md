# WIA-ROB-SEC-001: Security Robot
## Phase 3 — Decision & Mission

**Standard ID:** WIA-ROB-SEC-001
**Category:** Robotics — Security
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Purpose

Phase 3 governs how the world model produced in Phase 2 is turned into actions: which routes the robot patrols, how it responds to a detected anomaly, and how its authority to act is bounded. The phase places strong emphasis on *inspectability*: a security robot whose reasoning cannot be reconstructed after the fact is not a conformant security robot.

## 2. Mission Graph

A *mission* is the long-running plan that governs the robot's nominal behavior on a site. The standard requires the mission to be expressed as a directed graph of states, with explicit transitions:

```
MissionGraph := (V, E, v0, F)
    V  set of states (patrol leg, sentry post, recharge, maintenance)
    E  set of transitions, each guarded by a predicate over the world model
    v0 initial state (typically a depot or sentry post)
    F  set of terminal states (recharge, faulted)
```

The graph must be loaded from a signed file referenced in the conformance manifest. A platform must refuse to operate from an unsigned mission file. Field edits to the mission graph are themselves audit events and must be re-signed.

### 2.1 State Types

| State | Behavior |
|-------|----------|
| `patrol` | follow a route segment at declared speed and stand-off |
| `sentry` | hold position, scan declared cone, no locomotion |
| `respond` | move toward an event location with elevated alertness |
| `escort` | accompany a declared agent, holding declared offset |
| `recharge` | navigate to dock, idle until charged |
| `fault` | stop, illuminate, call operator |

`respond` and `escort` are higher-authority states. Entering them requires either the mission graph to permit them under the current conditions or an explicit operator command.

### 2.2 Transition Predicates

Predicates are declarative. The standard provides a small predicate vocabulary that conformant platforms must support:

```
PredicateVocabulary := {
    in_zone(z),
    track_present(class, zone, conf >= θ),
    sensor_health(s, status),
    operator_command(c),
    timer(name, op, value),
    battery(level op value)
}
```

A vendor may extend the vocabulary, but extensions must be declared in the conformance manifest so that an auditor can interpret the mission file without vendor-specific tooling.

## 3. Response Policy

### 3.1 Response Tiers

When the world model produces an anomaly, the response policy determines what the robot does. Response tiers are normative:

| Tier | Examples | Authority |
|------|---------|-----------|
| T1 — Observe | log event, increase scan rate | autonomous |
| T2 — Annunciate | turn on audible chirp, raise visible beacon | autonomous |
| T3 — Hail | play pre-recorded "you are entering a restricted area" message | autonomous, rate-limited |
| T4 — Engage | sustained light/audio, supervisor call | autonomous-with-confirm |
| T5 — Dispatch | request human security or law enforcement | operator-authorized only |

Tiers above T3 are not autonomous. T4 requires an operator confirmation within the configured window, otherwise the platform falls back to T3. T5 is operator-only — the robot may *recommend* T5 but may not invoke it.

### 3.2 Threshold Discipline

Each tier has a confidence threshold. Lower-confidence tracks unlock only lower-tier responses. A T3 hail driven by an unknown-class track at 0.4 confidence is non-conformant; the policy file must declare and the platform must enforce minimum confidences per tier.

A track must also persist for a minimum dwell time before any escalation tier may fire. Single-frame detections do not trigger Hail. The dwell budget is per-tier and is part of the policy file.

### 3.3 Inspectability

Every response action carries a *justification packet*: the policy node that fired, the world-model snapshot that satisfied its predicate, the per-track confidences at firing time, and the operator's confirmation token (for T4). The justification packet is part of the audit record. An auditor must be able to replay the policy against the snapshot and arrive at the same action.

## 4. Coordination with Humans

### 4.1 Personal Space

The platform must respect a declared minimum stand-off from any person-class track. Stand-off is part of the conformance manifest and is enforced at L1 by the platform's locomotion controller, not by L3 policy alone. An L3 plan that would violate stand-off is rejected by L1 and replaced by a slower, larger-radius alternative.

### 4.2 Right of Way

In any conflict between robot motion and human motion, the standard mandates yield-to-human. A platform that fails to yield is non-conformant. Yield does not require eye contact or recognition; the rule is over track presence, not track identity.

### 4.3 Signaling

The platform must signal its intent. A platform planning to move forward must visibly cue that intent (turn-signal-style indicator, audible "moving" tone) before motion begins. The cue is normative for nighttime and in any zone where ambient light is below the declared threshold.

## 5. Geofencing

The mission graph operates within a *geofence* declared in the conformance manifest. The platform must refuse to plan a path that exits the geofence, regardless of operator command, unless an operator with `safety_officer` authority signs a temporary geofence amendment. Such amendments expire at the configured horizon.

Geofencing is enforced redundantly: the global plan must respect the fence, and the L1 locomotion controller carries an independent fence check. A bug in the planner is not allowed to convert into a fence violation.

## 6. Multi-Robot Coordination

A site may operate multiple security robots. The standard recognizes two coordination modes:

### 6.1 Shared World Model

Robots may share their tracker outputs through a coordination service. Shared tracks are labeled with their originating platform; a track is not promoted to "corroborated" unless it is independently observed by a second platform's own sensors.

### 6.2 Mission Allocation

A coordination service may assign sentry posts and patrol routes across the fleet. Allocation messages are signed and auditable. A robot must reject an allocation that would force it outside its declared geofence or beyond its remaining battery envelope.

## 7. Battery and Endurance Policy

The policy file declares two battery thresholds:

| Threshold | Behavior |
|-----------|---------|
| `cautious` (e.g. 30%) | refuse new `respond` transitions; complete current patrol |
| `recall` (e.g. 15%) | abandon non-critical states; navigate to dock |

Below `recall`, only safety-critical states are honored. The standard prohibits "one more patrol leg" overrides at this level: the robot returns to dock.

## 8. Audit Hooks

Every state transition, every response action, every operator confirmation, and every fence amendment is an audit event. The Phase 4 specification describes the audit envelope in detail; here it suffices to say that the L3 layer is responsible for emitting these events with their justification packets, and that a conformant L3 must not have a code path that can take an action without emitting the corresponding audit event.

---

Phase 3 ends here. Phase 4 covers privacy guarantees, the audit envelope, the operator console, and lifecycle (commissioning, change, decommission).

## 9. Policy Graph Schema

A response policy is a labeled directed graph stored in a signed YAML or JSON document. The minimum schema:

```yaml
policy:
  id: "warehouse-night-v3"
  signed_by: "commissioner@deployment-id"
  signed_at: "2026-04-26T22:00:00Z"
  vocabulary: "wia-rob-sec-001/v1"
  nodes:
    - id: observe-track
      tier: T1
      requires: [ track_present(person, restricted_zone, conf >= 0.6) ]
      action: observe
    - id: hail-track
      tier: T3
      requires:
        - track_present(person, restricted_zone, conf >= 0.75)
        - dwell(track, t >= 4s)
      action: hail
      cooldown: 60s
    - id: engage-track
      tier: T4
      requires:
        - track_present(person, restricted_zone, conf >= 0.85)
        - dwell(track, t >= 8s)
      action: engage
      requires_confirmation_within: 5s
```

`vocabulary` pins the predicate vocabulary version so that a policy authored against an older vocabulary is interpreted under that vocabulary's semantics rather than silently rebound.

## 10. Replay Discipline

The audit log retains, for each fired policy node, a snapshot of the world model at firing time. An auditor can replay the policy file against the snapshot offline and must arrive at the same firing decision. A platform whose replay does not match its log entries is non-conformant: the gap between recorded and reproducible behavior is itself the harm.

## 11. Operator Override Discipline

An operator may override an autonomous policy decision in either direction: hold a fired escalation back to a lower tier, or escalate a non-fired situation to a higher tier. Both directions are audit events with the operator's identity, the original policy outcome, and the override rationale. An override does not modify the policy file — the file is authoritative; the override is a one-time situational deviation with its own justification.

Sustained override patterns indicate the policy file is mis-tuned. The standard recommends quarterly review of override frequency. A policy node whose decisions are routinely overridden in the same direction is a candidate for re-authoring.

## 12. Sensor-Mode Coupling

A response policy must declare how it behaves under perception modes other than `nominal`. Three behaviors are normative:

- **Strict.** Action is suppressed under any non-nominal mode.
- **Confirm.** Action proceeds under `degraded`, but T4 promotion now requires the operator regardless of dwell.
- **Block.** Action is unconditionally blocked under `stale` or `blind`.

A policy that does not declare its mode coupling is interpreted as `Block` under `stale` and `blind`, and `Confirm` under `degraded`. The default is conservative on purpose: a policy author who wants more aggressive behavior under degradation must say so explicitly, in writing, signed.

## 13. Time-Window Policies

A response policy may be conditioned on time of day or day of week. The policy file declares the windows in IANA time-zone form so that daylight-saving transitions are interpreted unambiguously. A policy condition that depends on UTC alone is acceptable but must be marked as such in the file; mixing local-clock and UTC predicates without explicit labels is non-conformant.

A platform that experiences a clock-drift incident during a time-window-conditioned policy must treat the window as undefined until the clock is re-anchored. The standard prefers no action over a wrong-time action.

## 14. Mission Resumption After Fault

A platform that exits the `fault` state must not silently resume the mission from the point of fault. The standard requires an operator review step: the platform reports the cause of the fault, the actions taken before the fault, and the proposed resumption point. Only after operator authorization does the mission resume.

Resumption authorization is itself an audit event. A pattern of repeated faults in the same mission segment indicates either a hardware issue or a mission-graph mismatch with the environment; both deserve operator attention rather than autonomous workaround.

## 15. Worked Policy Example

A retail concourse deployment specifies the following condensed policy for after-hours intrusion:

1. T1 fires on any person-class track in the concourse zone after 22:00 with confidence ≥ 0.6 — a logging tier, no audible or visual response.
2. T2 fires on the same track after 4 seconds of dwell — a soft chirp and ambient indicator.
3. T3 fires after 8 seconds with a hailing message identifying the zone as closed.
4. T4 is reachable only with operator confirmation; the policy node declares the confirmation window at 6 seconds with fallback to T3.
5. T5 (dispatch) is reachable only by direct operator action.

The policy declares its `Strict` posture under `stale`, `Confirm` under `degraded`, and time-window conditioning on the deployment's IANA zone. The signed file is referenced by hash in the conformance manifest. An auditor reading the manifest can locate, verify, and replay this policy against any incident in the audit log.

## 16. Anti-Patterns

The following patterns are explicitly non-conformant and must be rejected at policy review:

- A T4 escalation with no confirmation window or with a window long enough to be casual ("operator can confirm any time in the next 5 minutes"). The window is a live-supervisory affordance; it is non-conformant if it cannot serve that role.
- A response policy that increases the platform's authority in proportion to the operator's frustration ("after the operator overrides three T3s back to T1, the next track fires at T4"). Authority is not a debt to be repaid; the operator's overrides are a tuning signal, not a license.
- A response policy whose effect is to direct the platform into a posture that is not safe-by-default on policy disable. A policy whose disablement is a hazard is non-conformant: the platform must be safer with the policy off than on.

These anti-patterns appear in real deployments. The standard's policy review obligation is what catches them.
