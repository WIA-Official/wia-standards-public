# WIA-ROB-SVC-001: Service Robot
## Phase 4 — Operations, Accessibility & Lifecycle

**Standard ID:** WIA-ROB-SVC-001
**Category:** Robotics — Service
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Purpose

Phase 4 governs the human side of a service robot deployment: the operator console, accessibility obligations, multi-tenant venue coordination, and the platform's lifecycle from commissioning through field changes to decommissioning. Where Phases 1–3 describe what the robot does, Phase 4 describes the system the robot lives inside.

## 2. Operator Console

### 2.1 Required Surfaces

| Surface | Function |
|---------|---------|
| Live status | per-platform state, current task, perception mode |
| Engagement queue | active engagements and any escalated ones awaiting operator |
| Authorization bar | the affordance an operator uses for A3 confirmations |
| Audit search | replay any historical engagement from the audit store |
| Manifest viewer | read-only view of the active conformance manifest |
| Service-quality dashboard | refusal counts, escalation counts, accessibility metrics |

The console clearly indicates the active operator's authority. An operator without `safety_officer` privilege is visibly distinct from one with it.

### 2.2 A3 Confirmation Discipline

An operator confirmation for an A3-level task has a bounded window declared in the manifest. After the window, the platform falls back to refusal with a referral to the service owner. Confirmation must be a positive act — typing a token or pressing a hold-to-confirm control. Passive confirmation (close-the-popup-or-it-fires) is non-conformant.

### 2.3 Latency Budgets

The console must respect a declared end-to-end latency budget for safety-relevant operations. If the budget is violated for the active session, the platform falls back to a known-safe state on its own — the platform does not depend on the console for safety enforcement.

## 3. Accessibility Obligations

### 3.1 Profile Disclosure

The conformance manifest declares the platform's accessibility profile: which interaction modalities are supported, at what minimum, and where in the interface they are exposed. The profile is published in the deployment's public-facing materials. A member of the public must be able to know, before approaching, what the platform can and cannot do for them.

### 3.2 Required Modalities

The standard requires, at minimum:

- text-based equivalents of every voice prompt the platform issues,
- a tactile or audio readback for the public-facing identifier,
- non-voice path through every authorization that can be performed by voice,
- compatibility with personal assistive devices as declared in the profile.

A platform whose voice path is mandatory for any service is non-conformant: a deafened or non-vocal user must be able to receive the same service.

### 3.3 Complaint Pathway

The deployment publishes a complaint pathway routing through the service owner. The pathway must be reachable without using the platform itself. A user blocked by the platform's accessibility gap must be able to file a complaint by another channel.

### 3.4 Periodic Audit

The deployment audits its accessibility metrics periodically (cadence declared in the manifest, typically quarterly). Metrics include refusal rates by accessibility cue, completion times by cue, and complaints received. A pattern of disparity between cues is itself a finding.

## 4. Audit Envelope

### 4.1 Record Format

Each non-routine event is logged with:

```
AuditRecord := {
    record_id, platform_id, sequence, prev_hash,
    timestamp_local, timestamp_wall,
    layer, event_type, payload, justification,
    signature
}
```

The chain (`prev_hash` + `signature`) is verifiable end-to-end. A break in the chain is itself an incident.

### 4.2 Routine vs Non-Routine

A service robot generates many ordinary events (every transit, every greeting). The standard distinguishes:

- **Routine events** are aggregated and summarized; raw records are retained briefly within the manifest's routine-retention window. Aggregates are retained longer for service-quality review.
- **Non-routine events** — escalations, refusals, A3 authorizations, mode transitions, safety incidents — are individually retained in the chained store for the manifest's non-routine retention.

The distinction is normative: a deployment must declare its routine and non-routine retention windows in the manifest.

### 4.3 Storage Integration

Audit storage is integrated with WIA-SEC-017. The platform forwards records to the WIA-SEC-017 endpoint declared in the manifest. The endpoint stores records under its own integrity guarantees and signs receipts. A platform that cannot forward must buffer locally and continue to operate; once forwarding is restored, the buffered records are transmitted in order.

## 5. Multi-Tenant Venues

### 5.1 Tenant Boundaries

A service robot may operate in a venue with multiple tenants — a shopping mall, an airport, a hospital with several departments. The platform's geofence and authorization model must respect tenant boundaries.

A platform commissioned for tenant A may not deliver tenant B's services without a re-commissioning step that records both tenants' authorizations. The audit log preserves the tenant context of every engagement.

### 5.2 Shared Areas

In shared areas (corridors, atria, elevators), the platform follows the venue-wide rules (queue order, exit routes, quiet zones) regardless of tenant. The platform's task urgency is not a license to ignore venue-wide rules.

### 5.3 Cross-Platform Coordination

Multiple service robots may operate in the same venue, possibly for different tenants. Coordination follows the same model as in WIA-ROB-SEC-001 §6 with one additional requirement: a platform encountering another platform must not engage it in an interaction visible to the public. Two robots negotiating in a hallway over which goes first is permitted; two robots performing a scripted "look at me" interaction in front of guests is, by default, non-conformant unless the venue has authorized it as part of an exhibit or entertainment.

## 6. Commissioning

The minimum steps:

1. Verify firmware hash against the vendor's published value.
2. Load and sign the conformance manifest, including service classes, accessibility profile, service owner identity, and public-facing identifier.
3. Calibrate sensors; capture and sign the calibration report.
4. Run the conformance test battery (Phase 2 §10 and Phase 3 replay tests); capture and sign results.
5. Register the platform's hardware-rooted public key with the WIA-SEC-017 endpoint.
6. Bind operator, service-owner, and safety-officer credentials.
7. Issue a commissioning audit record.

A platform that has not been commissioned must refuse to enter an autonomous engagement state. It may be operated under direct teleoperation for the purpose of running the commissioning steps.

## 7. Field Change Control

### 7.1 Re-commissioning Triggers

The following changes require re-commissioning (a fresh manifest signature and a fresh audit anchor):

- firmware update,
- sensor swap or replacement,
- task graph change,
- interaction policy change,
- accessibility profile change,
- service-class change.

A change made without re-commissioning is, by construction, an incident.

### 7.2 Temporary Amendments

Real deployments need temporary changes — a single-day event with relaxed quiet-zone restrictions, a maintenance window with reduced engagement. Such changes are temporary amendments: signed by a safety-officer, expiring at a declared horizon, audit-logged. They must not silently extend.

### 7.3 Change Provenance

Every change to the manifest, task graph, interaction policy, accessibility profile, or geofence carries provenance: who signed it, against what prior state, with what justification. The provenance is queryable from the operator console.

## 8. Decommissioning

The minimum steps:

1. Issue a decommissioning audit record.
2. Revoke the hardware-rooted key from the WIA-SEC-017 endpoint.
3. Forward any cached audit records remaining in their retention window; destroy locally only if forwarding has failed permanently.
4. Wipe operator, service-owner, and safety-officer credentials and on-platform secrets.
5. Sign the decommissioning report.

The conformance test battery includes a decommission verification step.

## 9. Lifecycle Audit

The conformance manifest, the calibration reports, the conformance test results, the running task graph, the running interaction policy, the accessibility profile, the geofence, and the audit chain together constitute the *lifecycle audit* of a service-robot deployment. A conformant deployment must, on lawful request, produce the full lifecycle audit. A deployment that cannot is non-conformant by omission.

## 10. Service-Quality Review

Beyond the technical conformance audit, a deployment runs a periodic service-quality review (cadence declared in the manifest, typically monthly). The review covers:

- engagement completion rates,
- refusal rates and refusal reasons,
- escalation rates and escalation outcomes,
- accessibility metrics,
- user feedback received through the complaint pathway.

The review report is reviewed by the service owner and becomes part of the lifecycle audit. A deployment that does not run the review is non-conformant in operations even if every individual interaction is technically conformant.

## 11. Inter-Standard Dependencies

This standard depends on:

- WIA-SEC-017 (Security Audit) — audit transport and storage for non-routine events.
- WIA-ROB-001 (Robot baseline) — common robotic platform interfaces.
- WIA-ROB-SAFETY-001 (Robot Safety) — stand-off, yield, and interlock semantics.
- WIA-A11Y-001 (Accessibility baseline) — interaction-modality coverage and assistive-technology compatibility.

A deployment claiming WIA-ROB-SVC-001 conformance implicitly claims conformance to the relevant subset. A break in any dependency invalidates the WIA-ROB-SVC-001 claim until repaired.

## 12. Conformance Self-Test

A conformant deployment runs a self-test on each platform at a cadence declared in the manifest. The minimum self-test surface includes:

- e-stop response time within budget,
- audit chain unbroken since last anchor,
- calibration drift below threshold,
- accessibility-profile coverage (each declared modality is operational),
- venue-label enforcement working,
- console latency within budget.

A self-test failure is itself an audit event and may, by policy, take the platform out of service pending review.

## 13. Glossary

- **Accessibility profile.** The manifest's declaration of which interaction modalities the platform supports, used by the venue and the public to know what to expect.
- **Engagement.** Any interaction at I2 or higher — the platform has acknowledged a person.
- **Escalation.** A handoff from autonomous policy to a human (operator, service owner) when the policy cannot resolve the situation.
- **Lifecycle audit.** The complete record of a deployment, retrievable on lawful request.
- **Refusal.** A platform's neutral, named-accountable decline to perform a request.
- **Service owner.** The human accountable for the service the platform delivers, named in the manifest and in public-facing material.
- **Tier.** One of I0 through I5, ordered by interaction intrusiveness.

## 14. Disaster Recovery

A deployment must rehearse, at minimum annually:

1. **Audit endpoint outage.** Demonstrate that platforms continue to operate, buffer audit records, and flow them once the endpoint returns.
2. **Console outage.** Demonstrate that platforms fail safe on loss of console contact within the declared latency budget, and that the service-owner pathway remains reachable through alternative channels.
3. **Operator credential compromise.** Demonstrate the procedure for revoking and rotating an operator credential while platforms remain in service.
4. **Public-facing identifier compromise.** Demonstrate the procedure for retiring a compromised identifier (e.g., a defaced placard) and issuing a fresh one.

Rehearsal reports are signed by the commissioner and retained as part of the lifecycle audit (§9).

## 15. Interaction Telemetry

The console exposes per-platform interaction telemetry: tier distribution over a recent window, refusal rate, escalation rate, accessibility-cue distribution, and the queue of unresolved engagements. Telemetry is for service-quality review and operator oversight, not for individual surveillance — a person-by-person breakdown is not exposed in the telemetry surface.

## 16. Anti-Pattern Inventory (Operations)

The following patterns are non-conformant at the operations layer:

- **Authority bleed.** An operator without `safety_officer` privilege who can issue a stop-the-fleet command is a failure of the role model.
- **Silent retirement.** A platform decommissioned without a public notice — the deployment continues to advertise the service and routes to a robot that does not exist.
- **Dark-pattern complaint pathway.** A complaint pathway that requires a user to click through a satisfaction-rating UI before reaching a human is non-conformant. The complaint pathway must be a first-class affordance.
- **Hidden updates.** A firmware or policy update applied to deployed platforms without a re-commissioning step. Update equals re-commissioning equals fresh manifest signature.
