# WIA-infrastructure-monitoring PHASE 3 — PROTOCOL Specification

**Standard:** WIA-infrastructure-monitoring
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
infrastructure-monitoring programme: SHM-system commissioning,
calibration discipline, time-synchronisation discipline, threshold
governance, alert-response discipline, post-event capture protocol,
modal-analysis discipline, dam-instrumentation cycle, bridge-
instrumentation cycle, fibre-optic distributed-sensing governance,
data-stewardship through asset rehabilitation, and cybersecurity for
SHM acquisition systems.

References (CITATION-POLICY ALLOW only):

- ISO 13822 (assessment of existing structures)
- ISO 16587 (SHM performance parameters for fixed structures)
- ISO 4866 (vibration of fixed structures)
- ISO 18649 (mechanical vibration — evaluation of measurement
  results)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO 9001:2015 (quality management systems)
- ISO 31000 (risk management)
- ISO 8601 (date and time)
- ISO/IEC Guide 98-3 (uncertainty of measurement)
- IEC 62443 (industrial cybersecurity)
- IEEE 1451 (smart-transducer interface)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)

---

## §1 SHM-System Commissioning

A monitored asset is brought into the WIA-infrastructure-
monitoring programme through a documented commissioning
workflow:

- the asset owner authorises the SHM scope (which structural
  components are instrumented, which limit-states are watched);
- the SHM contractor publishes the sensor plan (locations,
  classes, expected ranges, anticipated failure modes
  monitored);
- sensors are installed, calibrated to traceable references,
  and their mountings (PHASE-1 §3) recorded;
- baseline campaigns capture the structure's behaviour under
  known reference conditions (ambient excitation, controlled
  load tests where applicable);
- threshold schemes (§4) are derived from the baseline and from
  the governing design code or owner-specific limit-state
  ladder.

Commissioning evidence is bound to the operator's quality-
management dossier so that downstream consumers can resolve any
monitored value back to the campaign that established its
baseline.

## §2 Calibration Discipline

Calibrations (PHASE-1 §4) are issued by ISO/IEC 17025-accredited
laboratories. Discipline requires:

- every sensor carries a current calibration record whose
  `validUntil` has not elapsed;
- expanded uncertainty is reported per ISO/IEC Guide 98-3
  (coverage factor, expanded uncertainty value and unit);
- traceability to a national metrology institute is recorded;
- recalibration cadence is set by the sensor class and the
  operating environment (more frequent for high-cycle
  accelerometers and for sensors exposed to harsh
  environments).

Calibration revocation triggers automatic suspension of derived
metrics that depend on the revoked sensor's data; the
operator's procedure register records the suspension policy.

## §3 Time-Synchronisation Discipline

Sensors used for dynamic monitoring (modal-frequency estimation,
vibration severity per ISO 4866 / 10816, blast monitoring) are
synchronised to a common reference whose precision is sufficient
for the analysis. Discipline requirements:

- NTPv4 stratum-2 or better (RFC 5905) is the floor;
- PTP (IEEE 1588) grandmasters are used when sub-microsecond
  precision is needed (multi-sensor modal analyses with
  closely-spaced modes);
- GNSS-disciplined oscillators are used when GNSS reception is
  reliable and a regional time base suffices;
- skew observations are recorded in PHASE-1 §5 records and
  drive priority-1 events when the per-sensor budget is
  exceeded.

## §4 Threshold Governance

Threshold schemes are bound to documented limit-state ladders:

- vibration severity thresholds derive from ISO 4866 (buildings)
  or ISO 10816 (rotating-machinery sub-assemblies on monitored
  structures);
- displacement and tilt thresholds derive from the asset's
  design-code limit-state values (CSA S6 for bridges, ICOLD
  guidelines for dams, FHWA guidance, owner-specific limits);
- corrosion-rate thresholds derive from the asset's design
  service-life model and the structural component's exposure
  classification;
- post-event thresholds (e.g. seismic-event-triggered
  permanent-deformation thresholds) derive from the design-
  earthquake intensity and the asset's design ductility.

Thresholds are reviewed at least annually by the operator's
SHM working group; revisions emit new thresholdScheme entries
in the operator's procedure register.

## §5 Alert-Response Discipline

Alerts (PHASE-1 §9) are routed to recipients per the operator's
on-call rota. Discipline requirements:

- maximum acknowledgement latency per severity (operator-
  declared);
- escalation paths when acknowledgement deadlines elapse;
- resolution evidence — inspection finding, restriction
  notice, or false-positive re-classification — is recorded
  before the alert can be marked resolved;
- false-positive re-classifications are reviewed in aggregate
  to drive threshold or sensor-placement revisions through
  change control.

## §6 Post-Event Capture Protocol

After a triggering event the operator triggers an automated
capture: the acquisition system extracts the time window
spanning the event and a configurable lead-in / lead-out
margin, archives the raw windows for all sensors on the
affected asset, and emits a post-event capture record (PHASE-1
§10). The capture is read by the post-event inspector
(PHASE-1 §5 of WIA-infrastructure) so that the inspection
finding is informed by the monitoring evidence.

## §7 Modal-Analysis Discipline

Modal analyses (natural frequencies, damping ratios, mode
shapes) are computed by published methods whose references are
recorded in the derived-metric record's `derivationMethod`
field (e.g. operational modal analysis using stochastic
subspace identification, eigensystem realisation algorithm,
frequency-domain decomposition). Method updates emit successor
metric records but do not retract earlier metrics.

## §8 Dam-Instrumentation Cycle

Dams classified as significant-hazard or high-hazard under the
operator's national dam-safety regulator's classification carry
SHM instrumentation per the regulator-mandated minimum (uplift
piezometers in the foundation, settlement points on the crest,
displacement targets on the downstream face for arch and
buttress dams, and seepage-flow gauges in the drainage
gallery). The operator's procedure register records the
inspection cadence for each instrument class and the regulator
deadline for reporting threshold breaches.

## §9 Bridge-Instrumentation Cycle

Bridges instrumented under the operator's bridge-safety
programme carry SHM per the owner's instrumentation plan; the
procedure register records the per-bridge sensor population,
the baseline campaigns, the routine inspection cadence, and
the post-event response procedure.

## §10 Fibre-Optic Distributed-Sensing Governance

Distributed-sensing campaigns (Brillouin or Rayleigh-based
distributed strain or temperature sensing) are governed by
fibre-installation records: every spliced fibre run is mapped
to its sensor identifier (PHASE-1 §2), every distributed
sample window cites the spatial-resolution and gauge-length
parameters used, and re-deployment of an interrogator unit on
a previously-installed fibre re-binds the sensor identifier
through a successor mounting record.

## §11 Data-Stewardship Through Rehabilitation

Asset rehabilitation events (PHASE-1 §7 work-orders in WIA-
infrastructure) typically remove and re-install sensors. The
SHM operator records the removal under PHASE-1 §11
decommissioning, the re-installation under a new sensor
identifier (or, when the same physical sensor is re-bonded,
the same identifier with a new mounting record), and the
baseline-recapture campaign that re-establishes thresholds.

## §12 Cybersecurity for SHM Acquisition Systems

SHM acquisition systems, edge gateways, and the operator's
analytics platform operate under the operator's ISO/IEC
27001:2022 information-security-management system. SHM
deployments on critical infrastructure are placed in IEC 62443
zones whose target Security Level reflects the asset's safety
class.

Firmware updates to acquisition systems are signed by the
vendor's release key and verified before install.

## §13 Quality-Management Dossier

The operator's quality-management dossier records the certified
ISO 9001 / ISO/IEC 27001 / ISO/IEC 17025 scope (where the
operator is the calibrating laboratory), the procedure
register, the threshold-scheme register, the analytics-method
register, and the deprecation history of SHM procedures. The
dossier is reviewed annually.

## §14 Cross-Border / Multi-Operator Programmes

Multi-jurisdiction operators honour each participating
jurisdiction's regulatory regime (dam-safety regulators,
bridge-safety regulators, occupational-safety regulators) and
record the applicable regulator references against each
monitored asset.

## §15 Programme Wind-Down

A programme that ceases operations transfers its records to a
recognised long-term archive, exports the procedure register
to the operator's records-management system, and notifies
regulators. Records subject to indefinite retention (post-
event captures, regulator-reportable threshold breaches)
transfer with content-addresses preserved.

## §16 Sensor-Plan Discipline

The sensor plan that the SHM contractor publishes during
commissioning (§1) is a normative artefact: it records the
locations selected for instrumentation, the sensor classes
chosen, the failure modes the plan anticipates, and the
analytics methods (§7) the plan supports.

Discipline requirements:

- the plan is signed by the SHM contractor's engineer-of-
  record;
- the plan is reviewed by an independent reviewer engaged by
  the asset owner;
- changes to the plan during the operating life of the
  programme go through change control, and superseded plan
  versions remain addressable for citation purposes;
- mounting records (PHASE-1 §3) cite the plan version that
  authorised the mounting.

## §17 Uncertainty Propagation Discipline

Derived-metric records (PHASE-1 §7) carry uncertainty estimates
where the operator's procedure register requires it (modal
frequency, displacement double-integration, corrosion-rate
estimation). Propagation discipline:

- input uncertainties are taken from the calibration records
  (PHASE-1 §4 expanded uncertainty);
- propagation method is recorded in the metric's
  `derivationMethod` field (e.g. GUM-compliant Monte-Carlo
  propagation per ISO/IEC Guide 98-3 / Supplement 1, linear
  combination of variances);
- uncertainty estimates are reported with the metric's
  reported value so downstream consumers do not interpret
  point estimates as exact.

## §18 Operator Training and Authorisation

Field technicians, SHM contractors' engineers, and the
operator's analytics team carry training and authorisation
records held in the operator's HR system and bound to the
sensor and analytics-method scopes they are authorised to
operate. The procedure register records the per-role
training requirements (bonded-strain-gauge installation
qualification, fibre-optic splice qualification, modal-
analysis certification, dam-instrumentation reader
certification) and the expiry policy for each.

Discipline:

- only authorised technicians sign mounting records;
- only authorised analysts sign derived-metric records;
- only authorised reviewers counter-sign threshold-scheme
  amendments under change control.

## §19 Conformance and Auditing

A programme conformant with WIA-infrastructure-monitoring
publishes its ISO 9001 / ISO/IEC 27001 certification references,
its programme registration in the discovery document, the
catalogue of sensors operated, the procedure register, and the
cyber-posture summary, and answers an annual self-assessment
that maps each clause of this PHASE to the programme's
implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-infrastructure-monitoring
- **Last Updated:** 2026-04-28
