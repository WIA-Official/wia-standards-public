# WIA-neuromorphic-chip PHASE 3 — PROTOCOL Specification

**Standard:** WIA-neuromorphic-chip
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
neuromorphic-chip programme: vendor accreditation, hardware-
description publication, model-support declaration, compiler
reproducibility, on-chip plasticity governance, characterisation
laboratory protocols, deployment certification, energy and
power-budget reporting, supply-chain provenance, and programme
wind-down.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17043:2010 (proficiency testing)
- ISO/IEC 27001:2022 (information security management)
- ISO 9001:2015 (quality management systems)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- IEEE Std 754-2019 (floating-point arithmetic; cited only for
  numerical-format definitions)

---

## §1 Vendor Accreditation

A neuromorphic-hardware vendor MAY claim conformance to
WIA-neuromorphic-chip after a recognised accreditation body has
issued a valid certificate against ISO 9001:2015 and against the
foundry-related cleanroom and yield-management requirements that
apply to the vendor's process. The accreditation register is exposed
to the API as a read-only resource.

A vendor whose accreditation is revoked deprecates all of its
hardware descriptions; deployed mappings remain operational but new
deployments cannot be compiled against the deprecated hardware.

## §2 Hardware-Description Publication

Hardware descriptions (PHASE-1 §4) are published with the vendor's
release-key signature attached. Each release is content-addressed in
the API and is reachable indefinitely so that compiled mappings
remain reproducible against the description they were compiled
against. Updates that change neuron or synapse model coverage emit a
new description rather than overwriting the prior one.

Vendors publish a deprecation timeline for each description. After
deprecation, the description remains addressable but new
compilations against it require justification recorded in the
mapping's metadata.

## §3 Compiler Reproducibility

A compiled mapping is reproducible when an independent invocation of
the compiler at the recorded version, given the same network
description, the same hardware description, and the same compiler
options, produces a mapping that is byte-identical or, where the
compiler is not deterministic by construction, equivalent under a
documented equivalence relation that the programme publishes.

Compilers that are not deterministic in their default mode SHOULD
provide a deterministic mode (typically by fixing the random seed)
and SHOULD record the seed in the mapping. Compilers that fall back
on a non-deterministic optimisation that the deterministic mode
cannot reproduce MUST flag the mapping as `non-deterministic` so
that downstream consumers can choose whether to consume it.

## §4 Plasticity Governance

Networks that learn online accumulate weight changes over their
deployment. The plasticity-event log (PHASE-1 §7) is the trajectory
of those changes. Programmes that publish externally cited learning
results MUST emit the plasticity log so that the trajectory is
auditable; aggregate-only emission is insufficient for citation.

For deployments in regulated contexts (medical-device decision
support, safety-critical robotics) the operating clinic or
robot-safety authority MAY require that plasticity be locked at
deployment time and unlocked only under controlled conditions; the
mapping's metadata records the lock state and the unlock authority.

## §5 Characterisation Laboratory Protocols

Reference laboratories follow ISO/IEC 17025:2017 procedures with
neuromorphic-specific extensions:

- **Voltage and current calibration** against traceable standards
  before each campaign.
- **Temperature control** of the device under test, recorded in
  the characterisation conditions.
- **Test pattern coverage** spanning the supported model parameter
  ranges; the test pattern is published so that consumers can
  reproduce the characterisation.
- **Yield-aware reporting**: the characterisation reports per-core
  variability statistics rather than chip-level averages so that
  downstream mapping can avoid out-of-spec cores.

## §6 Inter-Laboratory Round-Robin

Programmes that publish externally cited characterisation results
participate in inter-laboratory round-robin exercises at least once
every two calendar years. The round-robin protocol follows
ISO/IEC 17043:2010 expectations.

## §7 Energy and Power Budgets

Neuromorphic accelerators are typically deployed for their energy
efficiency. The programme records the deployed mapping's energy
budget (energy per inference, energy per spike-event, idle-power
floor) against the mapping. Programmes that publish externally
cited energy results MUST report energy under named workloads with
the workload definition published as a content-addressed artefact;
abstract energy claims without a workload definition are not
externally citable under this PHASE.

## §8 Records Retention

Programme records — every record defined in PHASE-1, the API audit
logs, mapping artefacts, characterisation evidence, plasticity logs,
and telemetry summaries — are retained for a minimum of seven
calendar years from the last access of the deployment. Externally
cited deployments retain indefinitely.

## §9 Time Synchronisation

Programmes operate on synchronised time per RFC 5905 (NTPv4) so that
events across vendor, laboratory, and integrator systems can be
ordered unambiguously.

## §10 Cybersecurity and Software Updates

Compiled mappings are signed by the SDK's release key and verified
by the deployed accelerator's host before execution. Firmware
updates to the host or the accelerator are signed by the
manufacturer's release key. Failed verification leaves the
deployment on the prior firmware and emits an alert through the
streaming endpoint defined in PHASE-2 §13.

## §11 Programme Wind-Down

A programme that ceases operations transfers indefinite-retention
records to a recognised long-term archive, notifies known external
citers via the well-known discovery document, and publishes a
sunset timeline for in-flight compilations. Externally pinned
evidence packages remain accessible through the archive.

## §12 Quality Dossier

The programme's quality dossier records the vendors it works with,
the hardware descriptions it tracks, the compiler revisions it
maintains, the round-robin exercises it has participated in, and the
deprecation history of its mappings. The dossier is reviewed at
least annually by the programme's quality manager.

## §13 Cross-Border Programme Operation

Programmes that operate across borders maintain a primary
jurisdiction of registration and one or more operating
jurisdictions. Cross-border data transfers honour the source-
jurisdiction's data-protection law for any deployments that consume
personal data (e.g. on-device learning over user-generated input).

## §14 Workload Selection and Reproducibility

Programmes that publish externally cited results select workloads
from a curated catalogue of community workloads or define new
workloads with sufficient detail that an independent group can
re-execute them. New workload definitions are content-addressed and
registered against the programme's catalogue. Re-execution by an
independent group on the same hardware-description version produces
results within the published reproducibility tolerance, which is
recorded in the workload definition.

A result that has not been re-executed by at least one independent
group is flagged in the public catalogue as `single-execution`; the
flag is removed when an independent re-execution is registered.

## §15 Plasticity-State Persistence and Recovery

Deployments that learn online persist plasticity state across power
cycles. The persistence protocol writes plasticity state through a
content-addressed snapshot whose digest is recorded against the
deployment. Recovery from a power cycle re-loads the most recent
verified snapshot; corruption of the snapshot triggers a fall-back
to the prior verified snapshot and emits an alert through the
streaming endpoint defined in PHASE-2 §13.

## §16 Energy-Reporting Honesty

Energy claims tied to externally cited results carry the measurement
method, the workload definition, and the laboratory or integrator
that performed the measurement. Claims that omit any of these are
not externally citable under this PHASE; the API enforces this by
refusing to include the claim in an evidence package.

## §17 Cybersecurity Threat Model

Programmes publish a threat model that names the trust boundaries
between the SDK, the host runtime, the accelerator, and the sensor
front-ends. The threat model identifies the assets to be protected
(plasticity state, weight matrices, AER traffic) and the controls
that protect them. The threat model is reviewed at least annually
by the programme's security officer.

## §18 Adversarial Input Considerations

Deployments that consume sensor input from unattended environments
(always-on wake-word listeners, event-camera-driven security systems)
are exposed to adversarial input. The deployment's risk file (PHASE-
3 §10 of an adjacent standard, when applicable) documents the
adversarial-input controls (rate limiting, statistical anomaly
detection, dual-path classification) that the deployment relies on.

## §19 Recipe Curation and Deprecation

Encoder and decoder recipes (PHASE-1 §12) are curated by the
operating programme. Each recipe is content-addressed at
registration; subsequent revisions emit new recipe identifiers
rather than overwriting prior ones, so that compiled mappings remain
reproducible against the recipe versions they were compiled against.

A recipe is deprecated through a published timeline (typically 12
months from announcement). After deprecation, the recipe remains
addressable but new compilations against it require justification
recorded in the mapping's metadata.

## §20 Cross-Vendor Compatibility Studies

Programmes that publish externally cited results across multiple
hardware platforms participate in cross-vendor compatibility studies.
The study evaluates whether the same network description, when
compiled and deployed across two or more vendor platforms, produces
results that agree within published tolerance. Compatibility studies
are coordinated by the certifying body or a community working group;
their outcome is recorded in the programme's quality dossier.

## §21 On-Device Learning and Privacy

Deployments that learn online over user-generated input (always-on
hearing aids that adapt to user environments, prosthetic controllers
that personalise to user motion) handle personal data. The operating
programme records the personal-data category at the deployment level
and applies the relevant data-protection law's controls (lawful
basis, retention bounds, subject rights). Plasticity events captured
during on-device learning carry the personal-data flag so that
exports honour the relevant constraints.

## §22 Conformance and Auditing

A programme conformant with WIA-neuromorphic-chip publishes its
accreditation certificate, its programme code registration, its
quality dossier, the catalogue of mappings it has compiled, and
the catalogue of characterisations it has performed, and answers
an annual self-assessment that maps each clause of this PHASE to
the programme's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-neuromorphic-chip
- **Last Updated:** 2026-04-27
