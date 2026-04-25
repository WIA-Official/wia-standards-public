# WIA-ROB-SEC-001: Security Robot
## Phase 4 — Operations, Privacy & Lifecycle

**Standard ID:** WIA-ROB-SEC-001
**Category:** Robotics — Security
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Purpose

Phase 4 governs the human side of the security robot: how operators interact with the platform, how privacy is preserved in the audit trail, how the platform is commissioned and decommissioned, and how field changes are controlled. Where Phases 1–3 describe what the robot does, Phase 4 describes the system around it.

## 2. Operator Console

### 2.1 Required Surfaces

A conformant deployment provides an operator console with at minimum the following surfaces:

| Surface | Function |
|---------|----------|
| Live feed | per-platform pose, current state, world-model summary |
| Incident queue | active and recent incidents, with justification packets |
| Confirmation bar | the affordance an operator uses to confirm a T4 escalation |
| Audit search | replay any historical incident from the signed audit store |
| Manifest viewer | read-only view of the active conformance manifest |
| Command console | issue authorized commands to the platform |

The console must show, at all times, the *authority* of the active operator. An operator without `safety_officer` privilege must be visibly distinct from one with it; this prevents an operator from believing they have authority they do not have.

### 2.2 Confirmation Discipline

An operator confirmation for a T4 escalation has a bounded window declared in the manifest. After the window, the platform falls back to T3 unconditionally. The window must be short enough to be live-supervisory (typically seconds, not minutes), and the console must visibly show the countdown.

A confirmation must be a positive act — typing a token or pressing a hold-to-confirm control. A passive confirmation (close-the-popup-or-it-fires) is non-conformant.

### 2.3 Latency Budgets

The console must respect a declared end-to-end latency budget for safety-relevant actions. If the budget is violated for the active session (network jitter, console freeze), the platform falls back to a known-safe state on its own — the platform does not depend on the console to enforce safety.

## 3. Privacy Guarantees

### 3.1 Minimization

A security robot's audit log is, by construction, a record of who-was-where. The standard requires three minimizations:

1. **No-recording zones** are honored at the platform layer (Phase 2 §5). The audit log carries a record of zone-blanking but no frames from inside the zone.
2. **Identity attribution** in the log is to track IDs, not to identified persons, unless an identification step has been authorized for the case at hand. A track is a track until an explicit authorized step says it is "Person X".
3. **Retention** is bounded. The manifest declares the retention period. Records past retention must be cryptographically destroyed (key destruction model: keys for a retention bucket are destroyed, rendering the encrypted records unreadable).

### 3.2 Subject Access

For any natural person whose track appears in the log, the operator must be able to produce, on lawful request, an extract of records concerning that person, along with the basis (which response policies fired, on which justification packets). The extract is itself audit-logged.

### 3.3 Demographic Auditing

The standard recommends, and a deployment in a privacy-regulated jurisdiction may require, periodic demographic auditing of the response-tier distribution: are escalations disproportionately falling on a class of people that should not, statistically, be the source of the events being responded to. The audit is deployment-level, not platform-level, but the platform must provide the data needed to perform it.

## 4. Audit Envelope

### 4.1 Record Format

Each audit record is a structured envelope:

```
AuditRecord := {
    record_id:        UUID,
    platform_id:      hardware-rooted identity,
    sequence:         monotonic per-platform counter,
    prev_hash:        hash of preceding record,
    timestamp_local:  monotonic clock reading,
    timestamp_wall:   synchronized wall clock reading,
    layer:            L1 | L2 | L3 | L4,
    event_type:       declared enum,
    payload:          event-specific structured data,
    justification:    reference to justification packet (Phase 3),
    signature:        platform signature over all preceding fields
}
```

The chain (`prev_hash` + `signature`) is verifiable end-to-end. The audit verifier rejects any sequence that does not chain cleanly. A break in the chain is itself an incident.

### 4.2 Storage

Audit storage is integrated with WIA-SEC-017. The platform forwards records to the WIA-SEC-017 endpoint declared in the manifest. The endpoint stores the records under its own integrity guarantees and signs receipts. A platform that cannot forward must buffer locally and continue to operate; once forwarding is restored, the buffered records are transmitted in order.

A buffer that fills past its declared capacity raises a non-suppressible alert. A platform may not silently overwrite older audit records in the buffer.

## 5. Commissioning

### 5.1 Steps

Commissioning is the act of turning a piece of hardware into a conformant deployment. The minimum steps:

1. Verify firmware hash against vendor's published value.
2. Load and sign the conformance manifest.
3. Calibrate sensors; capture the calibration report; sign it.
4. Run the conformance test battery (Phase 2 §7); capture results; sign them.
5. Register the platform's hardware-rooted public key with the WIA-SEC-017 endpoint.
6. Bind operator and safety-officer credentials.
7. Issue a commissioning audit record.

A platform that has not been commissioned must refuse to enter an autonomous state. It may be operated under direct teleoperation for the purpose of running the commissioning steps themselves.

### 5.2 Re-commissioning Triggers

The following changes require a re-commissioning step (a fresh manifest signature and a fresh audit anchor):

- firmware update,
- sensor swap or replacement,
- mission graph change,
- response policy change,
- geofence change beyond the temporary-amendment provision.

A change made without re-commissioning is, by construction, an incident: the platform's running manifest will not validate against the new state.

## 6. Field Change Control

### 6.1 Temporary Amendments

The standard recognizes that real deployments need temporary changes — a new construction zone, a one-day event with relaxed geofencing. Such changes are *temporary amendments*: signed by a safety-officer, expiring at a declared horizon, audit-logged. They must not silently extend.

### 6.2 Change Provenance

Every change to the manifest, the mission graph, the response policy, or the geofence carries provenance: who signed it, against what prior state, with what justification. The provenance is queryable from the operator console.

## 7. Decommissioning

A platform leaving the deployment must be decommissioned. The minimum steps:

1. Issue a decommissioning audit record.
2. Revoke the hardware-rooted key from the WIA-SEC-017 endpoint.
3. Destroy any cached audit records on the platform that remain inside their retention window (forward them first if forwarding is available; destroy them locally only if forwarding has failed permanently).
4. Wipe operator credentials and on-platform secrets.
5. Sign the decommissioning report.

A platform that has been decommissioned but not wiped is a residual risk. The conformance test battery includes a decommission verification step.

## 8. Incident Response Beyond the Robot

A security robot is one element in a security operations program. The standard does not specify the wider program, but it requires the following hooks:

- A documented escalation path for incidents the robot raises but cannot resolve (a T5-recommended event).
- A documented forensic procedure for retrieving and verifying audit records after a suspected tampering.
- A documented disclosure process for incidents involving members of the public — the standard does not mandate disclosure timelines (those are jurisdictional) but does require a procedure to exist.

## 9. Lifecycle Audit

The conformance manifest, the calibration reports, the conformance test results, the running mission graph, the running response policy, the geofence, and the audit chain together constitute the *lifecycle audit* of a security robot deployment. A conformant deployment must, on lawful request, produce the full lifecycle audit. A deployment that cannot is non-conformant by omission, regardless of how well its day-to-day operations behave.

---

This concludes Phase 4 and the Phase 1–4 normative content of WIA-ROB-SEC-001. The companion documents (`SPEC-APPENDIX.md`, `SPEC-GLOSSARY.md`) carry tables, schemas, and defined terms.

## 10. Console Telemetry

The console exposes per-platform telemetry to the operator. The required minimum surface includes pose, current state, perception mode, sensor health summary, battery level, current incident if any, and the active mission node. Telemetry must include a freshness indicator: an operator must be able to see at a glance that the displayed state is current. A frozen feed, presented as if live, is itself a safety hazard.

## 11. Disaster Recovery

A deployment must rehearse three scenarios at least annually:

1. **Audit endpoint outage.** Demonstrate that platforms continue to operate and buffer audit records, and that records flow correctly when the endpoint returns.
2. **Console outage.** Demonstrate that platforms fail safe on loss of console contact within the declared latency budget.
3. **Credential compromise.** Demonstrate the procedure for revoking and rotating an operator credential while platforms remain in service.

The rehearsal report is signed by the commissioner and retained as part of the lifecycle audit (§9).

## 12. Inter-Standard Dependencies

This standard depends on:

- WIA-SEC-017 (Security Audit) — audit transport and storage.
- WIA-ROB-001 (Robot baseline) — common robotic platform interfaces, where applicable.
- WIA-ROB-SAFETY-001 (Robot Safety) — stand-off, yield, and interlock semantics.

A deployment that claims WIA-ROB-SEC-001 conformance implicitly claims conformance to the relevant subset of these dependencies. A break in any dependency invalidates the WIA-ROB-SEC-001 claim until repaired.

## 13. Glossary Cross-Reference

The following Phase 4 terms are defined here and referenced from earlier phases:

- **Commissioning.** The act of binding a piece of hardware to a signed conformance manifest and registering its hardware-rooted key with the audit endpoint (§5).
- **Lifecycle audit.** The complete record of a deployment, from manifest through commissioning through every change and override and incident, retrievable on lawful request (§9).
- **Subject access.** The procedure for producing the records concerning an identified natural person at their lawful request (§3.2).
- **Temporary amendment.** A signed, expiring deviation from the manifest authorized by a safety-officer (§6.1).

## 14. Conformance Self-Test

A conformant deployment runs a self-test on each platform at a cadence declared in the manifest (typically daily). The minimum self-test surface includes:

- e-stop response time within budget,
- audit chain unbroken since last anchor,
- calibration drift below threshold,
- no-recording zone enforcement working under simulated entry,
- console latency to the platform within budget.

A self-test failure is itself an audit event and may, by policy, take the platform out of service pending review.
