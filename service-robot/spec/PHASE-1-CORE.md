# WIA-ROB-SVC-001: Service Robot
## Phase 1 — Core Architecture

**Standard ID:** WIA-ROB-SVC-001
**Category:** Robotics — Service
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Scope

A *service robot* under WIA-ROB-SVC-001 is a mobile or stationary platform whose operational duty is the delivery of a useful task to or alongside humans in non-industrial settings: hospitality, retail, healthcare reception, food delivery, cleaning, library and information service, museum and gallery interpretation, and similar. The standard normalizes the architectural layers, interaction contracts, and accountability guarantees required so that a single set of operating procedures applies across vendors and form factors.

Out of scope: industrial robots in fenced cells, surgical robots, fully autonomous on-road vehicles, security robots (covered by WIA-ROB-SEC-001), and weaponized platforms.

The defining characteristic of a service robot in this standard is that it *shares space with members of the general public who have not been specifically trained to interact with it*. This shapes every requirement that follows.

## 2. Service Classes

The standard recognizes the following service classes. A platform declares one or more in its conformance manifest, and the declared classes determine which behavior policies the platform is permitted to host.

| Class | Examples |
|-------|---------|
| Delivery | room service, document courier, retail aisle restocking |
| Hospitality | greeter, concierge, table-side ordering, navigation guidance |
| Information | wayfinding, FAQ kiosk-on-wheels, exhibit interpreter |
| Cleaning | floor cleaning, surface disinfection, waste collection |
| Companion | reading buddy, eldercare check-in (non-clinical) |
| Educational | classroom assistant, library helper |

A platform may host a behavior policy only for a class it has declared, and the policies are themselves bound to the conformance manifest. Cross-class deployment requires re-commissioning.

## 3. Layered Architecture

A conformant platform exposes four logical layers:

```
+----------------------------------------------------+
|  L4 — Operations & Service Lifecycle               |
|       (operator console, service supervision)      |
+----------------------------------------------------+
|  L3 — Interaction & Task Policy                    |
|       (HRI patterns, task graph, escalation)       |
+----------------------------------------------------+
|  L2 — Perception & Social Context                  |
|       (people detection, intent reading, accessibility) |
+----------------------------------------------------+
|  L1 — Platform & Safety                            |
|       (locomotion, e-stop, stand-off, payload)     |
+----------------------------------------------------+
```

Each layer has a normative interface and an advisory interface, with the same semantics as in WIA-ROB-SEC-001 §2: normative interfaces are required for conformance; advisory interfaces are documented hooks vendors may choose to expose.

### 3.1 L1 — Platform & Safety

L1 is the safety foundation. It owns:

- locomotion (drive, steering, lift if applicable),
- e-stop bus, physically independent of upper-layer software,
- payload monitoring (the platform must know what it is carrying and whether the payload is secure),
- self-test on power-on and during operation,
- compliance with declared stand-off distances during all motion.

Service robots routinely operate at distances where contact with a human is physically possible. L1 must enforce the stand-off envelope continuously, not as a periodic check.

### 3.2 L2 — Perception & Social Context

L2 produces the world model and adds a *social context* layer absent from purely industrial perception. Social context includes:

- person tracks with attention/intent signals (facing the robot, reaching toward it, walking past),
- queue and group structure (a line of people, a family group, a wheelchair user with a companion),
- accessibility cues (white cane, service animal, mobility aid),
- ambient noise level (informs voice volume and modality choice).

Social context is normative: a platform that can detect a service-animal cue but reports it as a generic "object" is non-conformant.

### 3.3 L3 — Interaction & Task Policy

L3 owns the platform's behavior repertoire: how it greets, how it offers help, how it executes its task while sharing space, and how it escalates to a human when its policy cannot resolve the situation. Policies are declarative and inspectable. An auditor must be able to read the policy node that produced any logged action.

### 3.4 L4 — Operations & Service Lifecycle

L4 is the human side: operator console, service-quality measurement, maintenance and recharge scheduling, and integration with the venue's information systems (point-of-sale, reservation, ticketing). L4 also owns the accessibility-complaint pathway.

## 4. Accountability Model

The standard recognizes four normative roles. A conformant deployment maps every actor to exactly one role.

| Role | Authority |
|------|----------|
| `operator` | day-to-day supervision; approves non-routine task variations |
| `commissioner` | signs the conformance manifest at deployment and on change |
| `service_owner` | accountable for the service the robot delivers; receives complaints |
| `safety_officer` | privileged: triggers fleet-wide stop; adjusts safety budgets |

`service_owner` is unique to service robots compared to WIA-ROB-SEC-001. A member of the public who is poorly served by the platform must be able to identify and contact a human who is accountable for the service. The service owner is named in the public-facing material and is the human who carries that accountability.

## 5. Threat and Misuse Model

Service robots operate in spaces where the threat model is dominated less by sophisticated adversaries and more by ordinary misuse, mischief, vulnerability, and accident. The standard declares the following classes:

| Class | Examples | Mitigation |
|-------|---------|-----------|
| Tampering | child climbs on the platform; load is removed mid-task | payload secure mode; child-aware stand-off; tamper alert |
| Misuse | operator uses the platform outside its declared class | manifest-bound policies; class-incompatible commands rejected |
| Failure | sensor fault, low battery, navigation loss | fail-safe defaults; clear visual signal of platform state |
| Accident | spill in front of platform; person falls nearby | platform stops, summons help, does not drive over hazards |

Adversarial threats (jamming, spoofing, credential theft) are still in scope but operate behind the dominant accident-and-misuse class.

## 6. Public-Facing Identity

Every service robot must carry a public-facing identifier visible to a member of the public. The identifier:

- is unique to the platform within the deployment,
- routes a complaint or query to the service owner,
- is readable by a sighted person and reachable by a person using assistive technology (printed identifier plus a tactile or audio readback affordance).

A platform without a public-facing identity is non-conformant: the standard's accountability model requires the public to be able to *find* the responsible party.

## 7. Conformance Manifest

The manifest carries:

- platform identity and firmware hash,
- declared service classes,
- declared sensor and effector envelope,
- the response/interaction policy graph,
- the service owner identity,
- the public-facing identifier,
- the accessibility profile (which assistive interactions are supported),
- the WIA-SEC-017 audit endpoint for non-routine events,
- the supervisory-handoff protocol.

The manifest is signed at commissioning. Field changes require a new signature.

## 8. Privilege Separation

Operator and safety-officer credentials are separate by construction. The same human may hold both, but the credentials and audit trail are distinct so a compromised operator credential cannot itself disable the safety pathway. Service owner is administratively separate from both: the human accountable for the service is not necessarily the human running it.

## 9. Versioning

The standard uses semantic versioning. A platform version identifier is part of every audit record so a record can be interpreted under its version's rules even if the deployment has since upgraded. Cross-major upgrades require explicit re-commissioning.

## 10. Document Conventions

**Must** carries normative force. **Should** indicates strong recommendation; deviations must be documented and justified in the manifest. **May** indicates a permitted option without preference.

## 11. Phase Roadmap

- Phase 2 covers perception and social context — what the platform must observe and how robustly.
- Phase 3 covers interaction patterns and task policies — how the platform decides what to do.
- Phase 4 covers operations, accessibility, multi-tenant venues, and lifecycle.

## 12. Service Owner Disclosure

A platform's service-owner identity is part of the conformance manifest and is published in human-readable form on the platform itself and in the deployment's public-facing materials. The disclosure includes a name (an organization is acceptable), a role title, and a non-platform contact channel (email, phone, web form) reachable without using the platform.

The standard requires the disclosure to be reachable by a person who has a complaint about the platform. A deployment that buries the contact, or that requires the user to interact with the platform to reach a complaint pathway, is non-conformant. Accountability is undermined when the path to it runs through the very system in question.

## 13. Anti-Pattern Inventory

The following patterns are explicitly non-conformant in the platform layer and must be rejected at commissioning review:

- An effector envelope including any harmful effector. The standard limits service-robot effectors to non-harmful surfaces (display, voice, light, gesture, mechanical pickup of declared payload classes).
- A manifest that declares a service class the platform's perception or interaction capabilities cannot support. A platform with no quiet-zone enforcement may not declare itself for hospital wards.
- A platform that re-uses session track IDs across deployments to recognize returning individuals without explicit opt-in. Cross-session re-identification is a separate, opted-in product feature, not a default.
- A platform that masks safety-relevant state (e-stop pressed, mode `blind`) under a friendly UI in the public-facing display. The platform's state must be legible to anyone who cares to look.

These are repeated in Phases 2–4 in their domain-specific forms.

## 14. Conformance Tiers

The standard recognizes three deployment tiers, declared at commissioning:

| Tier | Authorized Settings | Requirements |
|------|--------------------|----------------|
| Indoor-controlled | private offices, members-only clubs | declared-set service classes only |
| Indoor-public | malls, hotels, hospitals, libraries | full social-context perception; full accessibility profile |
| Outdoor-bounded | venue forecourts, outdoor terraces with declared geofence | weather and lighting envelope; geofence required |

A platform commissioned for one tier may not be redeployed into a higher-authority tier without re-commissioning. A redeployment into a *lower*-authority tier is permitted without re-commissioning since the lower tier's envelope is a subset, but the redeployment is itself an audit event.

A deployment that operates a platform outside its declared tier is non-conformant — most commonly, a platform commissioned for indoor-public hosting an outdoor walkup. The conformance manifest is what binds the platform to its tier; operating outside is operating without conformance.
