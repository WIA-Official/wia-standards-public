# WIA-photonic-chip PHASE 3 — PROTOCOL Specification

**Standard:** WIA-photonic-chip
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
photonic-chip programme: foundry accreditation, design house
qualification, PDK release-and-deprecation, tape-out submission and
acceptance, wafer-level test reproducibility, packaging certification,
deployed-module recall, laser-safety classification, supply-chain
provenance, and programme wind-down.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17043:2010 (proficiency testing)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO 9001:2015 (quality management systems)
- ISO 14644 (cleanrooms — air cleanliness classification)
- ISO 10110 (optics — preparation of drawings)
- ISO 8601 (date and time)
- IEC 60825-1 (laser safety — equipment classification)
- IEC 62496-2 (optical circuit boards)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- BIPM JCGM 100 (Guide to the Expression of Uncertainty in Measurement;
  cited normatively for terminology)

---

## §1 Foundry Accreditation

A foundry MAY claim conformance to WIA-photonic-chip after a
recognised accreditation body has issued a valid certificate against
ISO 9001:2015 (quality management) and ISO 14644 (cleanroom class
appropriate to the foundry's process). Each platform (silicon,
silicon-nitride, indium-phosphide, lithium-niobate-thin-film, etc.)
constitutes a distinct accreditation scope.

The accreditation register is exposed to the API as a read-only
resource and MUST be re-fetched at least once per calendar quarter.
A revoked or expired accreditation halts new fabrication-run
registrations from that foundry; runs already in-flight complete and
their outputs remain addressable.

## §2 PDK Release and Deprecation

Foundries publish process-design kits (PDKs) at versioned releases.
Each release is signed by the foundry's release key and is
content-addressed in the API per PHASE-1 §3. A PDK update that
changes a compact-model or a layout cell forces affected designs to
be re-built against the new PDK version; the API exposes a "PDK
diff" resource that lists affected designs so that design houses
can plan re-tape-outs.

Deprecation of a PDK version follows a published timeline (typically
12 to 24 months from the announcement). After deprecation, designs
pinned to the deprecated version MAY not be tape-out-accepted; they
are re-built against a supported PDK or marked as `deprecated`.

## §3 Tape-Out Submission and Acceptance

A tape-out submission includes the layout, the DRC and LVS reports,
the design's risk file (PHASE-3 §10), and the design house's QMS
certificate. Foundries verify that DRC and LVS report cleanly, that
the layout fits the reticle slot, and that the design house's QMS is
current. Submissions that fail any check are rejected with a Problem
Details (RFC 9457) response naming the check that failed.

Accepted submissions are scheduled into the foundry's wafer lot and
the foundry registers the fabrication run via the API on completion.

## §4 Wafer-Level Test Reproducibility

Wafer-level testing follows ISO/IEC 17025 procedures with
photonic-specific extensions:

- **Wavelength reference**: the test station's wavelength reference is
  calibrated against an iodine cell, an HCN gas cell, or an equivalent
  traceable reference at the start of each campaign.
- **Power calibration**: optical-power references are calibrated
  against a national-metrology-laboratory traceable standard.
- **Temperature control**: the device-under-test is held at a
  controlled temperature recorded in the measurement conditions; ±0.1 K
  is achievable on a Peltier-stabilised stage and is a reasonable
  default for ring and MZI characterisation.
- **Probe placement**: optical probes (fibre-array, grating-coupler
  fibre, edge-coupler V-groove) carry alignment-loss budgets that the
  measurement record reflects in its uncertainty contributions.

## §5 Inter-Laboratory Round-Robin

Programmes that publish externally cited per-component metrics
participate in inter-laboratory round-robin exercises at least once
every two calendar years per active platform. The round-robin
protocol follows ISO/IEC 17043:2010 and emits a public report that
the operating programme references in its quality dossier.

## §6 Packaging Certification

Packaging facilities operate under an ISO 9001-aligned QMS plus
IEC 60825-1 laser-safety classification competence. Each packaged
module's laser-safety class is verified against the module's optical
output power envelope and is recorded against the package record
(PHASE-1 §9). Packaged modules whose verification has not been
completed cannot be released for shipping; the API's package
endpoint refuses release and the facility's QMS records the
exception.

## §7 Deployed-Module Recall

A safety-relevant defect (laser-safety regression, packaging
delamination, fibre-attach reliability anomaly) triggers a recall
process that follows the operating jurisdiction's rules. The API
publishes a recall notice via the well-known discovery document
(PHASE-4 §5) and notifies system-integrator subscribers via the
streaming endpoint (PHASE-2 §13). System integrators acknowledge
the recall through a signed receipt; recall reach is computed from
the receipts.

## §8 Records Retention

Programme records — every record defined in PHASE-1, the API audit
logs (PHASE-2), DRC/LVS reports, raw wafer-test archives, component
measurements, and recall correspondence — are retained for a minimum
of seven calendar years from the last access of the design.
Externally cited designs retain indefinitely; on programme wind-down
the indefinite-retention records are transferred to a recognised
long-term archive.

## §9 Time Synchronisation

Programmes operate on synchronised time per RFC 5905 (NTPv4) so that
fabrication, test, and packaging events can be ordered unambiguously
across foundries, laboratories, and packaging facilities.

## §10 Risk and Hazard Files

Every design carries a risk file that documents the design's
intended use, foreseeable misuse, and the controls that mitigate
identified hazards. For modules whose deployed environments include
human exposure (display backlights, wearable sensors, biomedical
imaging), the risk file is more extensive and is reviewed by the
operating programme's safety officer.

## §11 Supply-Chain Provenance

Each fabrication run records the wafer lot's reticle source, the
photoresist vendor and lot, the dopant vendor and lot, and the
metallisation vendor and lot. Supply-chain provenance is preserved
so that vendor-side excursions can be traced to affected fabrication
runs and consequently to deployed modules.

## §12 Cybersecurity for Connected Modules

Photonic modules deployed in transceivers and sensors expose
management interfaces over mutually-authenticated TLS 1.3. Firmware
updates are signed by the manufacturer's release key and verified by
the module before install. Failed verification leaves the module on
the prior firmware and emits an alert.

## §13 Programme Wind-Down

A programme that ceases operations transfers indefinite-retention
records to a recognised long-term archive, notifies known external
citers via the well-known discovery document, and publishes a
sunset timeline for in-flight tape-outs. Externally pinned evidence
packages remain accessible through the archive at the same
content-addressed URLs.

## §14 Quality Dossier

The programme's quality dossier records the foundries it works with,
the platforms it operates on, the PDK revisions it tracks, the
round-robin exercises it has participated in, the recall history of
its modules, and the deprecation history of its designs. The dossier
is reviewed at least annually by the programme's quality manager
and is read during the annual ISO 9001 surveillance audit.

## §15 Cross-Border Programme Operation

Programmes that operate across borders maintain a primary jurisdiction
of registration and one or more operating jurisdictions where
fabrication, test, packaging, or deployment occurs. Cross-border
data transfers honour the source-jurisdiction's data-protection law,
which the operator records at fabrication-run registration time.

## §16 Co-Packaged-Optics and Heterogeneous-Integration Considerations

Modules that integrate photonic chips with electronic ICs in a single
package (co-packaged optics) carry additional coordination overhead:
synchronisation between photonic-chip yield and electronic-IC yield
during binning, thermal-management constraints that bound the
co-package's allowable operating temperature window, and reliability
testing that covers both die families. The programme records the
co-package family in the package record (PHASE-1 §9) and the
co-package's reliability test plan in the quality dossier.

Heterogeneous-integration platforms (III-V on silicon, lithium-niobate
thin-film bonded to silicon) follow the same conformance rules as
single-platform designs but emit additional records for the bonded
interface: bond-line topography, bond-strength characterisation, and
thermal-cycling reliability are recorded as auxiliary measurement
records.

## §17 Reliability and Burn-In

Photonic modules destined for high-reliability applications
(submarine cable, satellite payload, biomedical implant) carry
extended reliability and burn-in protocols beyond the foundry's
default qualification. The programme records the extended protocol's
identifier and the qualifying laboratory in the design's reliability
record. Modules that have not completed extended reliability are
flagged in the public catalogue so that high-reliability integrators
do not consume them in error.

Burn-in cadence is bounded by the laser-source's expected end-of-life;
burn-in conditions that accelerate end-of-life beyond the expected
margin MUST be approved by the operating programme's reliability
engineer before they are applied.

## §18 Eye-Safety Re-Classification on Update

Firmware or driver updates that change the module's optical output
power envelope MAY re-classify the module under IEC 60825-1.
Re-classification triggers a recall path under §7 if the new class
exceeds the certified deployment context; the API enforces the
recall path with type
`urn:wia:photonic-chip:laser-class-mismatch`.

## §19 Foundry Multi-Project-Wafer Coordination

Multi-project-wafer (MPW) tape-outs share a reticle slot among
multiple design houses. The MPW coordinator is the foundry; the
coordinator emits a per-MPW manifest that names the participating
designs and their reticle sub-slots. Per-MPW manifests are
attached to each design's evidence package so that downstream
auditors can verify that fabrication conditions were shared with
the named co-tenants.

## §20 Trade-Show and Demonstrator Releases

Demonstrator units released for trade shows, customer evaluation, or
third-party press review carry a `demonstrator` flag that disables
public-catalogue inclusion and limits the unit's downstream
integration to time-bounded evaluation. Demonstrators are recovered
or decommissioned at the end of the evaluation window per the
programme's quality dossier.

## §21 Conformance and Auditing

A programme conformant with WIA-photonic-chip publishes its
accreditation certificate, its programme code registration, its
quality dossier, the catalogue of designs it has fabricated, and the
catalogue of recalls it has issued, and answers an annual
self-assessment that maps each clause of this PHASE to the
programme's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-photonic-chip
- **Last Updated:** 2026-04-27
