# WIA-ROB-SEC-001: Security Robot
## Phase 1 — Core Architecture

**Standard ID:** WIA-ROB-SEC-001
**Category:** Robotics — Security
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Scope

A *security robot* under WIA-ROB-SEC-001 is an autonomous or supervised mobile platform whose operational duty is the observation, deterrence, and reporting of physical security events within a defined site boundary. The standard normalizes the architectural layers, interface contracts, and safety guarantees required so that a single operating procedure can be applied across vendors and form factors.

Out of scope: lethal autonomous platforms, weaponized units, and any platform whose default action set includes harm to humans. This standard explicitly limits effector authority to passive deterrence (lights, audio, recording) and to human-supervised intervention.

## 2. Layered Architecture

The conformant platform exposes four logical layers:

```
+----------------------------------------------------+
|  L4 — Operations & Reporting                       |
|       (incident creation, audit, supervisor UX)    |
+----------------------------------------------------+
|  L3 — Decision & Mission                           |
|       (patrol planning, anomaly response policy)   |
+----------------------------------------------------+
|  L2 — Perception & World Model                     |
|       (sensor fusion, tracking, semantic mapping)  |
+----------------------------------------------------+
|  L1 — Platform & Safety                            |
|       (locomotion, drive-by-wire, e-stop bus)      |
+----------------------------------------------------+
```

Each layer has a normative interface and an *advisory* interface. Normative interfaces must be present and correctly typed for conformance. Advisory interfaces are documented hooks vendors may choose to expose for richer integration.

### 2.1 Layer L1 — Platform & Safety

L1 is the lowest-trust layer of the upper stack and the highest-trust portion of the platform. It owns:

- locomotion control (chassis, drive, steering),
- hardware emergency-stop bus (independent of L2/L3 software paths),
- battery and thermal protection,
- self-test on power-on, periodic self-test in operation,
- watchdog timers governing every higher layer.

The e-stop bus is a normative requirement. Pressing any e-stop input must remove drive power within a bounded latency budget, regardless of the state of L2/L3. The latency budget is platform-specific but must be declared in the conformance manifest and must be physically realizable by hardware interlocks rather than software polling alone.

### 2.2 Layer L2 — Perception & World Model

L2 fuses raw sensor streams into a stable world model. The model contains:

- a metric occupancy representation of the local environment,
- a tracker over dynamic agents (people, vehicles, animals),
- a semantic overlay (door, window, restricted zone, emergency exit),
- a confidence channel attached to every track and semantic label.

Confidence is normative: any track delivered to L3 carries an explicit confidence in [0, 1]. L3 must respect confidence in its decision policy — for example, by requiring a higher confidence threshold for any escalation action than for routine logging.

### 2.3 Layer L3 — Decision & Mission

L3 turns the world model into actions. It owns the mission graph (patrol routes, sentry posts, response policies) and the policy that selects an action when an anomaly is detected. The policy must be declarative and inspectable: an operator must be able to read the policy that produced any given action, after the fact, from the audit trail.

### 2.4 Layer L4 — Operations & Reporting

L4 connects the robot fleet to the human security operations center. It owns the supervisor console, the incident lifecycle, and the audit log. L4 is normatively integrated with WIA-SEC-017 (security audit) for tamper-evident logging.

## 3. Threat Model

The standard defines a baseline adversary model so that every implementation begins from the same threat surface.

### 3.1 Adversary Capabilities

| Adversary | Capability | Goal |
|-----------|-----------|------|
| Local trespasser | physical proximity, no platform access | enter restricted zone undetected |
| Sophisticated intruder | jamming, spoofing GPS/Wi-Fi | disable detection or relocate the robot |
| Insider | network access, console credentials | suppress logs, rewrite policy |
| Supply chain actor | pre-shipment firmware modification | persistent backdoor |

A conformant deployment must show, in its threat model document, how each adversary class is addressed. Not every adversary must be fully mitigated — but unaddressed adversaries must be explicitly accepted in writing by the operator.

### 3.2 Defensive Posture

Defenses are layered:

1. **Tamper-evident logging.** All decisions, sensor snapshots at decision time, and effector activations are logged to a chained, signed store (WIA-SEC-017 conformant). Evidence is replayable.
2. **Out-of-band kill.** A physically independent kill channel exists that the operator can use to remove drive power without going through the network path.
3. **Fail-safe defaults.** Loss of network, loss of localization, low battery, or watchdog timeout drive the platform to a known-safe state (typically: stop, illuminate, raise visible/audible alert, transmit last known state).
4. **Mutual authentication.** Console-to-platform and platform-to-platform links are mutually authenticated. Single-sided TLS is non-conformant.

## 4. Conformance Manifest

Every conformant deployment publishes a manifest containing:

- platform identity (vendor, model, serial, firmware hash),
- e-stop latency budget and the test report demonstrating it,
- declared sensor suite and field-of-view envelopes,
- declared effector set (lights, audio, beacon — *no* harmful effectors),
- the response policy graph (machine-readable, inspectable),
- the WIA-SEC-017 audit endpoint and its public key,
- the supervised-handoff protocol used for any non-routine action.

The manifest is signed by the operator at commissioning. Any field change requires a new signature and is itself an audit event.

## 5. Identity & Roles

The standard defines four normative roles. A conformant deployment must map every actor in its administration model to exactly one of these roles.

| Role | Authority |
|------|----------|
| `operator` | day-to-day mission control; approves supervised handoffs |
| `commissioner` | signs the conformance manifest at deployment and on change |
| `auditor` | read-only access to the full audit trail |
| `safety_officer` | privileged: triggers fleet-wide stop; adjusts safety budgets |

Privilege separation between `operator` and `safety_officer` is normative: the same human may hold both, but the credentials and the audit trail must be distinct so that a compromised operator credential cannot itself disable the safety pathway.

## 6. Time and Identity Discipline

All audit records carry a monotonic local clock and a synchronized wall clock. Disagreement between the two beyond the configured drift envelope raises a non-suppressible alert, on the principle that a tampered clock is itself an incident.

Robot identity is bound to a hardware-rooted key. Software-only identities are non-conformant for the platform itself; they remain acceptable for transient session tokens issued from the hardware root.

---

This phase establishes the architectural and trust skeleton. Phase 2 covers perception and world-model construction; Phase 3 covers patrol and response policy; Phase 4 covers privacy, audit, and operator workflow.

## 7. Layer Boundary Enforcement

Cross-layer calls are gated. L1 must reject any L3 request that would bypass platform safety: a stand-off violation, a geofence exit, an over-speed in a declared low-speed zone. Rejection is itself an audit event so that an operator reviewing the log can see when L3 attempted a non-compliant action.

The layer boundary uses a small, declarative interface. The standard does not prescribe a transport (in-process, IPC, or networked are all permitted) but requires that:

- every cross-layer message is typed against a published schema,
- every cross-layer message carries a request ID that propagates to the audit record,
- every reject path produces a structured rejection that L3 can reason over rather than a silent drop.

## 8. Conformance Tiers

The standard recognizes three deployment tiers, declared at commissioning:

| Tier | Authorized Settings | Requirements |
|------|--------------------|----------------|
| Indoor-public | retail concourse, lobby | Tier-1 stand-off; T1–T3 only |
| Indoor-restricted | data center, archive | Tier-2 stand-off; T1–T4 with operator |
| Outdoor | site perimeter, yard | additional weather and lighting envelope; geofence required |

A platform commissioned for one tier may not be redeployed into a higher-authority tier without re-commissioning. A platform may be redeployed into a *lower*-authority tier without re-commissioning, since its conformance envelope strictly contains the lower tier's envelope; the redeployment is still an audit event.

## 9. Versioning and Compatibility

Within a major version, a platform is required to interoperate with operator consoles and audit endpoints across the same major version. Cross-major upgrades require explicit re-commissioning. The version identifier is part of every audit record so that, after the fact, a record can be interpreted under the rules of its own version even if the deployment has since been upgraded.

## 10. Glossary of Core Terms

The following terms appear throughout the standard with the meanings defined here:

- **Autonomous action.** An action the platform takes without per-event operator confirmation, drawn from a tier the platform is authorized to fire.
- **Conformance manifest.** The signed document declaring the platform identity, sensor and effector envelope, response policies, geofence, and audit endpoints. The platform's behavior is bound to its manifest.
- **Effector envelope.** The complete declared set of actions the platform can take on the world: lights, audio, beacons, recording. The standard prohibits effectors capable of harm.
- **E-stop bus.** The hardware emergency-stop pathway that physically removes drive power independent of any software path.
- **Geofence.** The polygonal region the platform is authorized to operate within, declared in the manifest.
- **Justification packet.** The bundled evidence (policy node, world-model snapshot, confidences, operator confirmation) recorded with each fired action so an auditor can reconstruct the firing decision.
- **No-recording zone.** A semantic-layer annotation under which recording-class sensors blank their persisted output while the detection signal still flows to the world model.
- **Operator confirmation.** A positive operator act (token entry or hold-to-confirm gesture) within a declared window that authorizes a T4-tier escalation.
- **Response tier.** One of T1–T5, ordered by authority: Observe, Annunciate, Hail, Engage, Dispatch.
- **Track.** A confidence-weighted observation of a dynamic agent in the world model, with provenance over the contributing sensors.
- **World model.** The fused, time-stamped representation of the local environment delivered by L2 to L3.

## 11. Document Conventions

Throughout this standard, **must** carries normative force: a deployment that does not satisfy the requirement is non-conformant. **Should** indicates a strong recommendation: a deployment that does not satisfy it must record the deviation and its justification in the manifest. **May** indicates a permitted option without preference.

Code samples are illustrative unless explicitly marked normative. Schemas are normative where so labeled and may be implemented in any encoding (YAML, JSON, CBOR) provided the canonical hash and signature still verify.
