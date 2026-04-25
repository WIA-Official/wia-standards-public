# WIA-MED-011: Smart Pill
## Phase 2 — In-Body Telemetry & Wireless Link

**Standard ID:** WIA-MED-011
**Category:** Medical — Ingestible Sensors
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Purpose

Phase 2 specifies how a smart pill's sensors produce data, how that data leaves the body, and how the external receiver decodes and validates it. The body is a hostile RF environment for the link physics and an unknown-mechanical environment for the platform. The standard's posture is that telemetry which cannot be timestamped, signed, and integrity-checked end-to-end is not telemetry; it is an unreviewable claim.

## 2. Sampling Discipline

### 2.1 Sampling Plans

A sampling plan declares, per sensor channel:

- nominal sampling rate,
- gating conditions (continuous, interval-triggered, event-triggered),
- calibration coefficients with provenance,
- maximum acceptable inter-sample gap.

The sampling plan is part of the conformance manifest. A platform may not silently change its plan in the field. A planned in-body change (e.g., increased sample rate after detected pH transition) is part of the plan when authored, not a runtime decision.

### 2.2 Calibration

Calibration coefficients are stored on the platform and signed at manufacture. The receiver verifies the signature on the first packet of a session and rejects the session if verification fails. A miscalibrated platform is, for clinical purposes, an unknown platform.

### 2.3 Sample Format

Each sample is encoded as:

```
Sample := {
    sequence:     monotonic per-session counter,
    sensor_id:    channel identifier,
    value:        raw measurement,
    units:        declared unit code,
    timestamp_l2: L2 monotonic clock reading,
    flags:        bitset (saturation, fault, calibration_pending),
    integrity:    signature or MAC over preceding fields
}
```

`integrity` is normative. A receiver presented with an unsigned sample treats it as advisory and labels it accordingly in the audit record. Clinical decisions must not be based on unsigned samples.

## 3. Wireless Link

### 3.1 Link Modalities

The standard recognizes three modality classes a smart pill may use to reach an external receiver:

| Modality | Typical Range | Notes |
|---------|---------------|-------|
| Galvanic conduction | through-body to skin patch | low energy; low data rate |
| Magnetic induction | near-field to a body-coupled receiver | medium energy; medium data rate |
| RF | through-body to an external antenna | higher energy; higher data rate |

A platform declares its modality in the manifest. The receiver must support the declared modality at the declared frequency or carrier. A platform whose link is not interpretable by the deployed receiver is, by construction, non-deployable.

### 3.2 Link Security

The link is encrypted under a per-platform symmetric key derived from the hardware root. The receiver shares the key, having been provisioned at commissioning. An attacker who captures the airwaves cannot decode the telemetry without the key.

A platform whose link is unencrypted is non-conformant for any deployment carrying patient-identifiable data, regardless of whether the deployment intends to encrypt downstream.

### 3.3 Loss Detection

The receiver detects gaps in the per-session sequence. A gap is a recorded fact: the audit record shows missed samples N..M. Gaps are not silently filled. A clinical reader must be able to see what was actually transmitted vs. what was inferred by the deployment.

### 3.4 Body-Coupling Variability

Coupling between the platform and the external receiver varies with patient posture, hydration, body composition, and proximate motion. The receiver must report a continuous link-quality indicator and must persist link-quality alongside each sample. A sample at low link quality is not invalidated, but its provenance includes the link-quality reading at receipt.

## 4. Energy Discipline

### 4.1 Budgeting

The platform's energy source supports a transit-duration envelope declared in the manifest. The duty cycle of telemetry, sensor sampling, and any actuation must fit within the envelope plus a declared safety margin.

A platform that exhausts before transit completion is non-conformant if the exhaustion stems from a misdesigned duty cycle. A platform that exhausts because the patient's transit took longer than the declared envelope is, instead, a transit-event finding, not a duty-cycle failure.

### 4.2 Failure Mode

On energy exhaustion the platform transitions to a passive state. It does not become an active hazard. Telemetry ceases. The capsule continues toward expulsion under L1's biocompatibility guarantees.

The receiver detects energy-exhaustion as the cessation of telemetry beyond the declared minimum continuous gap. Cessation is a clinical event and is recorded; it does not, by itself, constitute a non-conformance unless the cessation precedes the declared transit envelope by more than the configured margin.

## 5. Triggered Operations

### 5.1 Trigger Channels

A platform may take action on internal triggers (sensor crosses a threshold) or external triggers (receiver issues a command). External triggers are signed and replay-protected: a captured trigger may not be re-played to fire actuation again.

### 5.2 Triggered Actuation

For platforms in the Delivery class, a triggered actuation (controlled drug release) is the highest-stakes operation in scope of this standard. Such an actuation must:

- carry a justification packet (which trigger, with which evidence),
- be signed by the platform after firing with a freshly-derived key element,
- be reflected immediately in telemetry,
- be irreversible in the audit record (an actuation cannot be retroactively unfired).

A platform that fires an actuation but cannot prove it fired (because the supporting telemetry was not signed, or the audit chain was broken) is non-conformant.

### 5.3 Sampling Triggers

For platforms in the Sampling class, a triggered sample (localized GI fluid or tissue sample) is similar in stakes to actuation: the act is a one-shot consumable resource. The same audit and signing requirements apply.

## 6. Receiver State Machine

The external receiver has a small, declared state machine:

| State | Behavior |
|-------|---------|
| `idle` | not paired with a platform; awaiting administration |
| `armed` | paired with a platform manifest and key; awaiting first packet |
| `tracking` | receiving telemetry; buffering and forwarding to L4 |
| `gap` | telemetry interrupted past the configured threshold; alerting |
| `complete` | telemetry has ceased and the transit envelope has elapsed |
| `fault` | platform-side fault detected; clinical alerting |

Transitions are audit events. The receiver does not transition out of `armed` without a successfully-decoded first packet, and does not transition out of `tracking` to `complete` until the transit envelope has elapsed plus the declared margin.

## 7. Conformance Tests

The platform's Phase-2 conformance is demonstrated by:

- a benchtop telemetry test exercising the declared modality and confirming sample-integrity verification,
- an in-vitro link-quality test under a body-coupling phantom,
- a duty-cycle test demonstrating that the energy budget is respected within the transit envelope,
- a key-rotation test demonstrating per-platform key uniqueness,
- a triggered-operation test (for Delivery and Sampling classes) demonstrating signed-actuation audit integrity.

Test reports are signed by the commissioner and attached to the conformance manifest.

## 8. Privacy Posture

In-body telemetry contains physiologic data of a specific person. The standard's posture is conservative:

- the link is encrypted end-to-end under per-platform keys,
- the receiver does not aggregate or transmit data without explicit clinician authorization,
- raw telemetry is retained only within the deployment's clinical data store under WIA-SEC-017 audit,
- aggregated statistics for service quality are computed under privacy-preserving aggregation; per-patient time series do not leave the deployment without explicit authorized release.

A deployment that streams raw telemetry into a third-party analytics service without explicit clinician and patient authorization is non-conformant in privacy regardless of the analytics service's claims.

## 9. Worked Example — Adherence Confirmation

A patient ingests an Adherence-class platform alongside their medication. The platform's L2 stack samples a digestive-environment indicator (pH, gastric activation marker) at 1 Hz for the first 30 minutes. The first sample crossing the gastric-activation threshold is the adherence confirmation event. The platform signs the event, transmits it via galvanic conduction to the skin patch (L3), and continues sampling for the remaining transit envelope.

The receiver decodes the activation event, verifies its signature, marks the session as `confirmed`, and forwards the event packet to L4. The clinical workflow at L4 records the adherence confirmation in the patient's medication-management timeline. The session record retains both the activation event and the surrounding sample stream so that, on review, a clinician can confirm the activation came from genuine ingestion and not from a spoofed packet.

The replay invariant: an auditor can take the session record, re-verify the platform key, re-verify the activation signature, and reproduce the confirmation decision. Replay mismatch is itself an incident.

## 10. Anti-Patterns

- A telemetry stream presented to the clinician without the gaps. Gaps are first-class data; hiding them is non-conformant.
- A receiver that "smooths" cessation events by inferring continued sampling from prior values. The receiver reports cessation; it does not invent samples.
- A platform whose telemetry is unsigned in any deployment carrying patient-identifiable data.
- A platform whose declared modality the deployed receiver does not support. The deployment is non-deployable; releasing it to a patient is a misidentification waiting to happen.

## 11. Time Synchronization Protocol

L2 carries a monotonic clock; L3 carries a wall clock. At session start, the receiver records the wall-clock time at which the first signed sample arrives and the L2 sequence and timestamp on that sample. Subsequent samples are mapped from L2-monotonic to wall-clock by linear interpolation against this anchor, with a drift correction recomputed at each cleanly-decoded sample.

A platform whose wall-clock-mapped sample timestamps drift beyond the configured envelope across a session is, post-hoc, marked as a clock-drift event in the session record. Drift events do not invalidate the data but inform the clinical reader that exact timing of in-session events should be treated with the recorded uncertainty.

## 12. Sample Validity Rules

A sample is *valid* for clinical decisions if:

1. its signature verifies under the session key,
2. its sequence number is consistent with prior samples in the session (no jumps without a recorded gap),
3. its calibration timestamp is within the declared shelf life of the calibration coefficients,
4. its link-quality at receipt is at or above the declared minimum for clinical-decision use.

A sample failing any rule is recorded but labeled in the audit log as the rule it failed under. A clinical decision made on a sample failing rule (1) or (2) is non-conformant; on rules (3) or (4), it is permitted under documented operator override with the override audit-logged.
