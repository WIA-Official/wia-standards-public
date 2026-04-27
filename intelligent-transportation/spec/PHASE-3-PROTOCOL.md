# WIA-intelligent-transportation PHASE 3 — PROTOCOL Specification

**Standard:** WIA-intelligent-transportation
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
intelligent-transportation programme: regulator authorisation,
type approval of RSU and OBU radios, V2X security credential
management, privacy filtering of broadcast records, mutual-aid
governance with neighbouring TMCs, transit-priority and emergency
preemption discipline, incident-management workflow, decommissioning
of roadside infrastructure, and records retention.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO 14001:2015 (environmental management)
- ISO 39001:2012 (road traffic safety management)
- ISO 21217:2020 (CALM ITS station architecture)
- ISO 21384-3 (UAS — operational procedures, cited where TMC
  programmes interact with traffic-monitoring drone operations)
- ISO/IEC 17025:2017 (calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- IEEE 1609.2-2022 (V2X security services)
- ETSI TS 103 097 (security headers and certificate formats)
- ETSI TS 102 941 (security credential management)
- SAE J2945/1 (V2V minimum performance requirements)
- SAE J3161/1 (C-V2X)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- TMDD v3.1 (mutual-aid messaging conventions)
- NTCIP 1202 v03 / 1203 v03 (signal controller / DMS protocols)

---

## §1 Regulator Authorisation

A road operator MAY claim conformance to WIA-intelligent-
transportation only after the national radio regulator has
authorised the programme's RSU radio operations under the
applicable spectrum band (5.850–5.925 GHz ITS band in most
jurisdictions, with regional variations on DSRC vs C-V2X
allocation).

The programme records the spectrum authorisation reference per
band per jurisdiction. Authorisations that revoke or narrow are
recorded; the API enforces the revocation by refusing to
register an RSU whose radio stack is no longer authorised.

## §2 Type Approval

RSU and OBU radios carry type-approval references against the
applicable conformance regime (FCC OET Bulletin 65 / Part 95 in
the US, RED Directive equivalents in the EU, regional regulators
elsewhere). Type-approval revisions emit cross-references when
they change power masks, channel allocations, or coexistence
behaviour.

## §3 V2X Security Credential Management

V2X messaging is authenticated under the Security Credential
Management System (SCMS) per IEEE 1609.2 / ETSI TS 102 941.
Programmes configure:

- **Enrolment authority** that issues the long-term enrolment
  certificate to each OBU and RSU.
- **Pseudonym certificate authority** that issues short-lived
  pseudonym certificates to OBUs (preserving driver privacy by
  rotating the broadcast certificate frequently — typically every
  5 minutes).
- **Misbehaviour reporting service** that consumes signed
  reports from peers about anomalous V2X behaviour and decides
  on certificate revocation.
- **Certificate revocation list (CRL) distribution** for
  revoked certificates.

Misbehaviour reports flow through the operator's incident-
management workflow when they correlate with infrastructure
faults; they flow to the SCMS operator when they implicate
OBU misbehaviour.

## §4 Pseudonym Rotation Policy

Passenger-vehicle OBUs rotate pseudonym certificates at the
cadence the regulator's privacy framework requires (5 minutes
is the SAE J2945/1 default). Transit-vehicle and emergency-
vehicle OBUs MAY operate under longer-lived role certificates
since their operational role is itself disclosed by virtue of
the vehicle's external markings.

Rotation cadence is recorded against the programme; rotations
that fall outside the published cadence are flagged in the
operator's audit log.

## §5 Privacy Filtering of Broadcast Records

V2X capture archives (PHASE-1 §6) contain broadcast messages that
were public over-the-air but that, when aggregated and stored,
allow reconstruction of vehicle trajectories. Programmes apply a
privacy filter before any disclosure outside the programme:

- BSM messages are de-aggregated so that no consumer outside the
  programme can reconstruct individual vehicle journeys longer
  than the operator's privacy threshold (typically 60 seconds).
- Pseudonym certificates within the archive are replaced with
  capture-local opaque tokens.
- Geographic precision is reduced to the operator's published
  privacy precision (typically 10-metre cells for non-incident
  research, 1-metre cells for incident investigation under court
  order).

The filter version applied is recorded against every disclosure;
filter upgrades are versioned independently of the API.

## §6 Mutual-Aid with Neighbouring TMCs

Adjacent road operators exchange incident, detour, and signal-
coordination information across boundaries through the integration
described in PHASE-4 §6. Mutual-aid governance:

- Both parties carry mutual-aid agreements that name the data
  classes shared and the response-time commitments.
- Cross-boundary detour plans (a detour on operator A's network
  that touches operator B's signals) require operator B's
  acknowledgement; absent acknowledgement, the detour falls back
  to within-network only.
- Joint-operations exercises are conducted at least annually.

## §7 Transit-Priority and Emergency Preemption

Transit-signal-priority (TSP) and emergency-vehicle preemption
(EVP) are governed by the operator's preemption policy, recorded
against the programme. Preemption requests carry the requesting
vehicle's role token and are validated against the operator's
authorisation register; unauthorised preemption requests are
logged and discarded.

Programmes that operate cross-jurisdictional transit (regional
bus systems crossing TMC boundaries) record reciprocal
authorisation so that a transit vehicle on a partner network is
treated identically.

## §8 Incident-Management Workflow

Incident workflow follows a documented playbook per incident
classification. Incidents at severity `critical` (multi-fatality
collision, hazmat spill, infrastructure collapse) trigger the
operator's emergency-operations centre activation.

Root-cause investigations for incidents that cause public-safety
impact are deposited as evidence and surface in the operator's
annual safety report.

## §9 Decommissioning of Roadside Infrastructure

RSU decommissioning follows the operator's asset-disposition
process: physical removal, certificate revocation through the
SCMS, archival of the RSU's operational history (firmware
versions, message volumes, fault history), and update of the
programme's geographic scope. Decommissioned RSUs remain
addressable in the API at their content-addressed identifier so
that historical incident investigations can reference them.

## §10 Records Retention

Programme records — every record defined in PHASE-1, the API
audit logs, the V2X capture archives at their declared retention
window, the regulator submissions, and the mutual-aid exchanges —
retain per the regulator's required window. Public-safety-impact
incident records and their root-cause investigations retain
indefinitely.

## §11 Time Synchronisation

Programme clocks synchronise per RFC 5905 (NTPv4) against a
national-metrological-laboratory stratum-1 service or an
equivalent recognised service. Sub-microsecond timing required
for V2X interlocks (intersection movement assist, signal-phase
coordination across nearby intersections) uses PTP or GNSS-
disciplined oscillators per the operator's timing dossier.

## §12 Cross-Jurisdictional Programme Operation

Multi-jurisdiction programmes (interstate corridor management,
trans-European corridor management, border-crossing freight
corridors) honour each participating jurisdiction's spectrum
rules, type-approval rules, and privacy framework. Per-segment
records carry the governing jurisdiction.

## §13 Quality Dossier

The programme's quality dossier records the regulator
authorisations, the mutual-aid agreements, the SCMS operator
relationship, the pseudonym rotation cadence, the privacy filter
versions, the joint-operations exercises performed, and the
operational events the programme has experienced. The dossier
is reviewed at least annually by the operator's quality
manager.

## §14 Safety Management

Programmes that operate under a road-safety management framework
(ISO 39001:2012 or equivalent) integrate the WIA-intelligent-
transportation records with the operator's safety dossier:
safety performance indicators (KSI rates, secondary-crash rates),
safety risks identified through V2X data analytics, and the
corrective actions undertaken.

## §15 Cybersecurity for Operational Technology

TMC operational technology (signal controllers, RSU back-ends,
preemption gateways) follows the operator's defence-in-depth
cybersecurity expectations recorded in the operator's IEC 62443
zone classification. Cybersecurity incidents that affect
operational technology trigger an immediate notification under
the operator's incident-response process.

## §16 VRU Privacy and Aggregation

Vulnerable-road-user awareness records (PHASE-1 §10) carry
de-identified trajectories produced by the operator's privacy
filter. The filter applies the same precision-reduction rules as
the V2X capture filter (§5) and additionally suppresses
trajectories shorter than the operator's published minimum
threshold (typically 30 seconds) to prevent re-identification of
individual pedestrians at low-traffic locations.

Aggregate VRU detection counts published through the API
(PHASE-2 §18) are subject to a cohort-size protection rule that
suppresses counts in cells where fewer than the operator's
threshold of detections occurred.

## §17 Work-Zone Activation Discipline

Work-zone records (PHASE-1 §11) are authorised by the operator's
construction-permit system; activation in the API requires the
permit reference. Work zones that activate without a permit
reference (emergency lane closures for collision response, debris
removal) carry a `provisional` flag and reconcile to a permit
reference within 24 hours; failure to reconcile triggers an
audit alert.

Cross-boundary work zones (a work zone whose geometry crosses
TMC boundaries) require coordination with the partner TMC
through the mutual-aid integration of §6.

## §18 Vehicle-Class Authorisation

Vehicle-role tokens recorded in the OBU record (PHASE-1 §5)
authorise the OBU to participate in role-specific protocols:
transit-signal-priority for transit OBUs, emergency-vehicle
preemption for emergency OBUs, snowplow-coordinated signal
adjustment for snowplow OBUs. Role assignments are subject to
the operator's authorisation register; vehicles whose role
authorisation has expired are demoted to baseline OBU operation.

## §19 Conformance and Auditing

A programme conformant with WIA-intelligent-transportation
publishes its spectrum and type-approval references, its mutual-
aid agreements, its quality dossier, and the catalogue of
material incidents (severity `critical` and resolved
public-safety incidents), and answers an annual self-assessment
that maps each clause of this PHASE to the programme's
implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-intelligent-transportation
- **Last Updated:** 2026-04-28
