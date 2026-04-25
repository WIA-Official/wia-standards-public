# WIA-ROB-SVC-001: Service Robot
## Phase 3 — Interaction & Task Policy

**Standard ID:** WIA-ROB-SVC-001
**Category:** Robotics — Service
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Purpose

Phase 3 governs how a service robot turns the world model and social context into actions: how it greets, how it executes a task, how it shares space, how it escalates to a human when its policies cannot resolve a situation. The phase emphasizes legibility — a service robot whose intent is illegible to a member of the public is, regardless of its technical sophistication, a poor service robot.

## 2. Task Graph

A service robot's nominal behavior is a *task graph* — a directed graph of states with explicit transitions, signed and bound to the manifest. The task graph is structurally similar to the mission graph in WIA-ROB-SEC-001 §2 but its state types are different.

| State | Behavior |
|-------|----------|
| `idle` | hold position at a designated stand, conserve energy |
| `transit` | move along a planned route to a next service location |
| `engage` | offer or deliver service to a person or group |
| `wait` | hold for a person to act (entering elevator, accepting payload) |
| `recover` | self-recovery from a stuck or degraded state |
| `recharge` | navigate to dock, idle until charged |
| `fault` | stop, signal, summon help |

The task graph is signed and any field edit requires re-signing. A platform refuses to operate from an unsigned task graph.

## 3. Interaction Tiers

A service robot's interaction with a person is tiered by intrusiveness. The standard requires the platform's policy to make the tier explicit at every interaction so that the audit log records which tier fired.

| Tier | Intrusiveness | Examples |
|------|--------------|----------|
| I0 — Ambient | platform present but not addressing the person | passing by, executing transit |
| I1 — Available | platform signals availability without addressing | beacon, soft idle animation |
| I2 — Acknowledge | platform notes presence without engaging | brief greeting in response to attention |
| I3 — Offer | platform offers service | "Hello, may I help you find your gate?" |
| I4 — Deliver | platform is executing requested service | wayfinding, delivery, information |
| I5 — Escalate | platform routes the situation to a human | "Let me get my colleague to help with that" |

The tiers progress from least to most intrusive. A platform must not skip from I0 to I3 without intermediate cues; the absence of intermediate cues is itself a form of being startled-by, which the standard treats as a service failure.

## 4. Engagement Discipline

### 4.1 Attention-Driven Engagement

A platform may engage at I3 only when the world model reports a person at attention ≥ θ for at least the configured dwell. The dwell defends against "passing-by" engagement (a person walks past, the platform pivots and offers help, the person was not asking). Such defensive engagement is non-conformant if it occurs systematically.

### 4.2 Voice Volume and Modality

Voice is calibrated to ambient noise. In a `quiet_zone`, voice output is suppressed and the platform falls back to screen-based interaction. In a high-noise environment, voice output may be raised — but only up to the declared envelope. A platform that defeats a quiet-zone restriction by going louder is non-conformant.

A platform that has detected a deafness cue (hearing aid, signing) must offer text or sign-language interaction surface where supported. The accessibility profile in the manifest declares which alternatives are available.

### 4.3 Stand-off and Posture

The platform's stand-off envelope is declared in the manifest. During engagement, the platform respects:

- a minimum stand-off below which it will not approach,
- a comfortable stand-off at which it offers service,
- an escalated stand-off when the person has shown they prefer distance (turning away, raising a hand, stepping back).

The platform reads the escalated stand-off cue and steps back. A platform that does not step back when a person steps back is non-conformant.

### 4.4 Service-Animal and Mobility-Aid Etiquette

When an `accessibility_cues` track includes `service_animal`, the platform does not engage the animal — no voice directed at it, no eye-level visual attention. When the cue includes a mobility aid (white cane, wheelchair, walker), the platform extends its stand-off envelope and slows its motion in the person's vicinity.

## 5. Task Authorization

### 5.1 Service Authorization

A service robot delivers tasks. Some tasks require authorization: opening a lockbox, releasing a package to a specific recipient, taking payment. The authorization model is a small set of normative levels.

| Level | Trigger | Mechanism |
|-------|---------|-----------|
| A0 | none required | greet, wayfind, deliver public information |
| A1 | recipient acknowledgement | "Are you Pat Lee?" — verbal confirmation |
| A2 | structured token | scan a guest's room key, scan a delivery code |
| A3 | operator confirmation | live human authorization, time-bounded |

A service robot may not autonomously upgrade an authorization level. A platform that decides it has "enough confidence" to skip from A2 to A3 is non-conformant.

### 5.2 Refusal Discipline

A service robot must support graceful refusal: a person asks for something the platform cannot or will not do. The refusal must:

- name the human accountable (the service owner or operator) so the person can take the request to them,
- be neutral in tone (not apologetic-blaming the person, not implying the person should not have asked),
- be logged for service-quality review.

Refusal logs are reviewed periodically. A class of refusal that turns out to be overly common indicates a policy gap rather than a user error.

## 6. Sharing Space

### 6.1 Yielding

In any conflict between robot motion and human motion, the platform yields. Yielding is over track presence, not track identity. The platform does not require the person to negotiate.

### 6.2 Doorways and Elevators

The platform must not block doorways, exit routes, or elevator entries. In an elevator shared with humans, the platform takes the smallest plausible footprint, holds in a corner, and signals that it is waiting. Joining a shared elevator at peak occupancy is permitted only if the venue policy explicitly allows it; the default is no.

### 6.3 Queues

Where the venue layer marks a `queue zone`, the platform respects queue order. A platform that wedges into the front of a queue because it has a more urgent task is non-conformant. The platform's task may be urgent; that does not relieve it of the queue.

### 6.4 Crowding

In dense crowd conditions, the platform reduces speed and may pause altogether rather than push through. The standard prohibits "intent forcing" behavior — visibly nudging a slow person, edging into a person's path to compel motion. Forcing is non-conformant; waiting is conformant.

## 7. Escalation to Humans

### 7.1 Triggers

The platform escalates to a human when:

- the policy cannot resolve the request,
- the person has indicated dissatisfaction,
- a vulnerability or distress cue has appeared,
- a safety condition has occurred (a spill, a fall),
- the platform has been in a degraded mode beyond its threshold,
- the operator console has lost contact beyond the latency budget.

### 7.2 Mechanism

Escalation is more than logging an event. The platform must ensure a human receives the escalation and can engage in time. The mechanism includes:

- a routed notification to the operator (with venue-appropriate priority),
- a fallback to the service owner if the operator does not engage within the configured window,
- a visible signal to the person — they should know help is on the way, not just that the robot stopped serving them.

### 7.3 Containment

While awaiting human engagement, the platform holds in a contained, non-blocking position. It does not continue task execution as if nothing had happened, and it does not abandon the person.

## 8. Replay Discipline

Every fired interaction tier and every escalation carries a justification packet. An auditor must be able to replay the policy file against the world-model snapshot and arrive at the same firing decision. A platform whose replay does not match its log entries is non-conformant.

## 9. Worked Example — Hotel Concierge

A concierge platform is in `idle` at the lobby stand. A guest approaches with attention ≥ 0.7 for 2 seconds. The platform fires I2 (acknowledge) with a screen smile; the guest stops. The platform fires I3 (offer) with "Hello, can I help?" The guest asks for the way to the spa. The platform consults the venue layer, plans a route, and fires I4 (deliver) by leading the guest at a paced 0.4 m/s, with periodic glance-back animations.

Halfway, the guest stops to take a phone call. The platform reads the attention drop, holds, and shifts to a `wait` state. After the configured timeout it gently signals "I'll wait whenever you're ready." When the guest resumes, the platform continues. At the destination it fires the closing I3 ("The spa is just here. Anything else?") and returns to its stand.

Every transition is logged with its justification packet; a reviewer can replay the encounter offline.

## 10. Anti-Patterns

The following are non-conformant:

- skipping interaction tiers (I0 → I3 with no intermediate cue),
- escalating in response to operator frustration rather than situational evidence,
- "dark patterns" (intentionally ambiguous refusal, or interaction that leads the user to a paid upgrade they did not ask for),
- voice that defeats a quiet zone by going louder,
- engaging service animals or addressing children at intrusive tiers without parental presence and authorization.

## 11. Policy Schema

A service-robot interaction policy is a labeled directed graph stored in a signed YAML or JSON document. Minimum schema:

```yaml
policy:
  id: "lobby-greeter-v2"
  signed_by: "commissioner@deployment-id"
  signed_at: "2026-04-26T22:00:00Z"
  vocabulary: "wia-rob-svc-001/v1"
  nodes:
    - id: greet-attended
      tier: I2
      requires:
        - person_track(intent in {approaching, attending}, attention >= 0.7, dwell >= 1s)
      action: soft_welcome
    - id: offer-help
      tier: I3
      requires:
        - person_track(attention >= 0.8, dwell >= 2s)
        - venue_layer.zone == public
      action: voice_offer("Welcome, can I help?")
      cooldown_per_track: 30s
    - id: complete-task
      tier: I4
      authorization: A1
      action: deliver_service
    - id: escalate-distress
      tier: I5
      requires:
        - person_track(cue.distress == true)
      action: route_to_service_owner
```

`vocabulary` pins the predicate-vocabulary version. `cooldown_per_track` defends against the platform asking the same person five times if they linger.

## 12. Mode Coupling

A policy must declare its behavior under non-nominal perception modes. Three behaviors are normative:

- **Strict.** Tier suppressed under any non-nominal mode.
- **Confirm.** Tier permitted under `degraded` only with operator-style oversight.
- **Block.** Tier unconditionally blocked under `stale` or `blind`.

Default if undeclared: `Block` under `stale` and `blind`, `Confirm` under `degraded`. A policy that wants more aggressive behavior must say so explicitly, signed.

## 13. Time-Window Conditioning

A policy may be conditioned on time of day or day of week. The condition is in IANA time-zone form. A policy whose time predicate depends on UTC must be marked as such.

## 14. Multilingual Policies

Where the deployment serves a multilingual public, the platform's voice and screen output is selected from a set of declared languages. The language selection mechanism is part of the policy: detected on first interaction (greeting in the venue's primary language with a "select language" affordance), or triggered by an operator setting, or chosen explicitly by the user. A platform that defaults to one language and offers no path to another is non-conformant if the deployment is in a venue serving a multilingual public.
