# WIA-infrastructure-monitoring PHASE 4 — INTEGRATION Specification

**Standard:** WIA-infrastructure-monitoring
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited infrastructure-monitoring
programme integrates with the systems that surround it: the asset
owner's WIA-infrastructure asset record, the operator's WIA-
infrastructure-integration federation, sensor vendor support
portals, ISO/IEC 17025 calibration laboratories, post-event
inspection workflows, dam-safety and bridge-safety regulator
notification intakes, the operator's CMMS / EAM, the operator's
historian archive, long-term archives, and citation tools that
resolve published monitoring reports to their evidence packages.
It also defines the evidence-package format that bundles a sensor's
or asset's complete monitoring record set for external audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO 13822 (assessment of existing structures)
- ISO 16587 (SHM performance parameters)
- ISO/IEC 17025:2017 (calibration accreditation)
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- IEC 62443 (industrial cybersecurity)
- IEEE 1451 (smart-transducer interface)
- OGC SensorThings API 1.1
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Asset-Record Linkage (WIA-infrastructure)

Every sensor record (PHASE-1 §2) carries a `parentAssetRef`
that resolves to a WIA-infrastructure asset identifier. The
linkage is bidirectional: the WIA-infrastructure asset's
timeline endpoint (PHASE-2 §3 of WIA-infrastructure) lists
sensors associated with the asset, and the WIA-infrastructure-
monitoring sensor's timeline links back to the asset.

Linkage submissions whose `parentAssetRef` does not resolve in
the asset programme's discovery document return
`urn:wia:infrastructure-monitoring:asset-not-found`.

## §2 Federation Linkage (WIA-infrastructure-integration)

Operators that publish sensor telemetry into a multi-system
integration federation register their telemetry flows in the
federation's message-flow catalogue (WIA-infrastructure-
integration PHASE-1 §7). The flow record cites the sensor
identifier(s) and the operator's quality-of-service contract.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  sensors/                     — sensor records and decommissioning
  mountings/                   — mounting history
  calibrations/                — calibration certificates and chain
  time-syncs/                  — time-sync history
  sample-windows/              — envelope records and archive
                                  references
  derived-metrics/             — derived-metric series
  threshold-breaches/          — breach records
  alerts/                      — alert records (acknowledged /
                                  resolved)
  post-event-captures/         — capture records
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed by
the SHM operator and counter-signed by the asset owner when the
package supports a regulatory submission.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the manifest,
and reject the package on mismatch with type
`urn:wia:infrastructure-monitoring:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-infrastructure-monitoring` that links to the
API root, the operator's certifications, the procedure register,
and the catalogue of monitored assets.

## §6 Sensor-Vendor Support Portal Integration

Sensor vendors operate support portals that consume failure
escalations when a sensor's behaviour deviates from its
published specification. The integration carries the vendor's
portal URI, the support-contract reference, and the per-sensor-
class escalation paths.

## §7 Calibration-Laboratory Integration

ISO/IEC 17025-accredited calibration laboratories emit signed
calibration certificates that bind the sensor identifier to a
traceable national-metrology reference. The integration carries
the laboratory's accreditation reference, the calibration
certificate's content-address, the issuing date, and the
validity expiry. Laboratories that revoke a certificate (e.g.
because of a discovered systematic error) emit revocation
records that the operator's monitoring programme picks up via
the streaming subscription.

## §8 Post-Event Inspection Workflow Integration

Post-event captures (PHASE-1 §10) are read by the asset owner's
post-event inspector (PHASE-1 §5 of WIA-infrastructure). The
integration record carries the captured-window content-
addresses and the captured-sensor list so that the inspector's
finding cites the monitoring evidence available at the time
of inspection.

## §9 Regulator Notification Integration

Dam-safety regulators, bridge-safety regulators, and
critical-infrastructure regulators consume threshold-breach
notifications and post-event capture notifications. The
integration record carries the regulator's identifier, the
notification format, the maximum notification latency, and the
acknowledgement workflow.

## §10 CMMS / EAM Integration

The operator's CMMS or EAM consumes resolved alerts and
post-event captures to schedule follow-on inspection or
maintenance work. The integration is bidirectional: alerts
emitted by the SHM programme open follow-on inspections in the
CMMS, and CMMS work-order completion bound to the asset
re-baselines the SHM thresholds when the work changed the
structural response.

## §11 Historian Archive Integration

The operator's historian archive holds raw-sample windows
(PHASE-1 §6) for the period required by the asset's
regulatory regime. The integration record carries the
historian's identifier, the per-sensor retention period, the
storage tier, and the retention deletion policy when the
retention period elapses.

## §12 Long-Term Archive Integration

Programmes designate a long-term archive that holds records
beyond programme wind-down. Quarterly deposits round-trip
content-addresses; on wind-down, remaining records transfer to
the archive. Recognised archives include national archives,
professional-society archives, and trustworthy digital
repositories accredited under ISO 16363 or equivalent.

## §13 Worked Example: Citation Resolution for a Bridge Modal Report

A reader encounters a peer-reviewed publication that cites the
operator's modal-frequency monitoring of a long-span bridge.
The reader's tool resolves the citation by:

1. Parsing the citation to extract the asset identifier and
   manifest digest.
2. Fetching the discovery documents for the WIA-infrastructure
   asset programme and the WIA-infrastructure-monitoring
   programme.
3. Resolving the manifest URLs and verifying the manifest
   signatures.
4. Recomputing the manifest digest and comparing it to the
   pinned digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests;
   surfacing the resolved evidence (sensors, calibration chain,
   sample windows, derived modal frequencies, threshold
   schemes) to the reader.

A conformant tool completes this flow without further input.

## §14 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (infrastructure
for asset records, infrastructure-integration for federation
flows, intelligent-transportation for traffic operations on
monitored bridges, electric-grid for monitored transmission
infrastructure) emit cross-standard linkage records.

## §15 Reader Tooling

Programmes MAY publish supplementary reader hints (visual
sensor-location plots on the asset's BIM model, modal-shape
animations, threshold-trend charts) alongside the canonical
evidence package. Reader tools are non-normative.

## §16 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose attestations (ISO/IEC 17025
calibration accreditation, ISO/IEC 27001 certification, SHM
contractor qualifications) to consumers of W3C Verifiable
Credentials MAY re-issue the attestations as Verifiable
Credentials under the Data Model 2.0 specification. Re-issuance
is optional; the canonical record remains the JSON evidence-
package manifest.

## §17 Streaming Heartbeat

SSE subscribers (PHASE-2 §15) receive a heartbeat every 30
seconds with `Last-Event-ID` resume support; subscribers that
disconnect resume without losing visibility of priority-1
breach or alert events.

## §18 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window
of at least one full ISO/IEC 27001 surveillance cycle.

## §19 Sensor-Population Public Catalogue

Programmes that publish a public catalogue of monitored assets
emit a feed listing assets with their evidence-package manifest
digests, the sensor populations, and the threshold-scheme
references. The feed does not carry raw sample data; it is
intended for transparency to the public served by the asset.

## §20 Field-Engineer Mobile Integration

Field engineers travelling to monitored assets use mobile
applications to record installation evidence, perform routine
sensor checks, and capture remediation evidence after an alert.
The integration carries the mobile application's identifier,
the per-sensor field-procedure reference (calibration check,
torque check on welded mounts, fibre-end-face inspection for
fibre-optic interrogators), and the offline-capture fallback
that survives loss of connectivity at remote dam, bridge, and
levee sites. Captured evidence rounds-trip into the operator's
record set when connectivity is restored.

## §21 Real-Time Operations Console Integration

Asset owners' operations consoles (bridge-traffic operations
centres, dam-control rooms, transit operations centres)
consume threshold-breach and alert events for situational
awareness. The integration emits a real-time event stream
(over the Server-Sent Events endpoint of PHASE-2 §15 or via the
operator's federation broker) and a status panel that displays
sensor health, calibration status, and alert posture per
monitored asset.

## §22 Sensor-Health Trend Service Integration

Programmes that operate large sensor populations (hundreds to
thousands of sensors across many assets) integrate with a
sensor-health trend service that ranks sensors by drift
indicators (calibration deviation, time-sync skew, output-
range saturation, noise-floor degradation). The integration
emits a per-sensor health rank and a recommended action (in-
field check, recalibration, replacement). Recommendations
feed the operator's CMMS as candidate work orders and remain
under the operator's review before being authorised.

## §23 Time-Sync Reference Source Integration

Programmes operating PTP-grandmaster or GNSS-disciplined-
oscillator references for sub-microsecond timing emit reference-
source health events (oscillator hold-over duration, GNSS-fix
quality, holdover allan-deviation). The integration record
carries the reference source's identifier, the calibration
chain to the national time standard, and the fallback rules
when the primary reference becomes unavailable.

## §24 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with the asset programme (WIA-infrastructure), at least one
ISO/IEC 17025 calibration laboratory, the relevant regulator's
notification intake, the operator's CMMS or EAM, the operator's
historian archive, and at least one long-term archive, and has
published at least one externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-infrastructure-monitoring
- **Last Updated:** 2026-04-28
