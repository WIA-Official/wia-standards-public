# WIA-infrastructure-integration PHASE 3 — PROTOCOL Specification

**Standard:** WIA-infrastructure-integration
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
infrastructure-integration federation: federation governance, naming
authority, change-control board, semantic-mapping discipline,
protocol-binding discipline (OPC UA, IEC 61850, IEC 60870-5-104,
DNP3, BACnet, MQTT, AMQP, Kafka), namespace stewardship, replay-
window discipline, cybersecurity (IEC 62443 zones, IEC 62351 secure
substation communications), data-stewardship through vendor
upgrades, regulator-driven audits for critical-infrastructure
operations, and programme wind-down.

References (CITATION-POLICY ALLOW only):

- ISO 15926 (life-cycle integration)
- ISO 16739 (IFC)
- ISO/IEC 27001:2022 (information security management)
- ISO 9001:2015 (quality management systems)
- ISO 31000 (risk management)
- ISO 8601 (date and time)
- IEC 61850 (substation automation)
- IEC 62351 (security for power-system communications)
- IEC 62443 (industrial communication networks — cybersecurity)
- IEC 60870-5-104 (telecontrol over TCP/IP)
- IEC 62541 (OPC UA)
- IEC 61968 / 61970 (CIM)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)

---

## §1 Federation Governance Charter

A federation MAY claim conformance to WIA-infrastructure-
integration only after the operating organisation has published a
governance charter at the content-address recorded in PHASE-1 §2.
The charter records:

- the operating organisation and its accountable executive;
- the naming authority for cross-system identifiers;
- the change-control board's membership and terms of reference;
- the dispute-resolution procedure when contributing systems
  disagree on the canonical value of a shared data element;
- the federation's scope (which systems are in, which are out);
- the certification posture each contributing system must hold
  to be onboarded.

Charters are amended through the change-control process (§5).
Charter amendments emit audit-trail records (PHASE-1 §9) so the
charter's revision history is auditable end-to-end.

## §2 Naming Authority

The federation's naming authority is the single source of truth
for cross-system entity identifiers. Naming authority duties:

- maintain the namespace mapping (PHASE-1 §6);
- adjudicate disputes between contributing systems whose local
  identifiers refer to the same physical asset under different
  names;
- publish the resolver endpoint;
- review and approve namespace-revision change-control records.

A federation that operates without a named naming authority has
no path to PHASE-3 conformance because the namespace inevitably
fragments along vendor and team boundaries.

## §3 Change-Control Board

The federation's change-control board approves changes to schemas,
protocol bindings, semantic mappings, namespace mappings, and
system onboarding / decommissioning. Membership at minimum
includes:

- the federation's lead integration architect;
- a representative of each major contributing system's operations
  team;
- the operating organisation's information-security lead (for
  changes that affect IEC 62443 zone boundaries);
- a regulator-facing representative (when the federation is
  operated under critical-infrastructure regulation).

The board's approval procedures are recorded in the governance
charter (§1).

## §4 Semantic-Mapping Discipline

Semantic mappings (PHASE-1 §5) are the federation's *meaning*
contracts. Discipline requires:

- every mapping cites the source-system schema reference and the
  target-system schema reference exactly (OPC UA NodeId, IEC
  61850 Logical Node, CIM 16 class, IFC entity identifier, etc.);
- mappings are validated by an integration architect and
  counter-validated by a reviewing architect before they are
  marked production-grade;
- ML-classifier mappings publish their training-set provenance
  and their offline validation accuracy;
- deprecated mappings remain addressable under their identifier
  for citation purposes; production traffic moves to the
  successor mapping under change control.

## §5 Change-Control Procedure

Changes proceed through a documented sequence:

1. *Proposal* — change record created with `proposedAt`.
2. *Impact assessment* — affected flows, mappings, and systems
   are listed; downstream consumers are notified.
3. *Approval* — board members append signatures (§3 quorum).
4. *Rollout window* — change is applied during the planned
   window; rollback plan is loaded into the broker's standby
   path.
5. *Verification* — integration architect verifies that the
   change behaves as specified.
6. *Closure* — change record is marked completed and audit
   trails are sealed.

Failed verifications trigger the rollback plan; the change record
captures the rollback evidence.

## §6 Protocol-Binding Discipline

Each contributing system's protocol binding (PHASE-1 §4) is
governed under the IEC / OPC standards that apply:

- *OPC UA*: IEC 62541 conformance with the vendor's published
  profile; security per IEC 62541-2 with mutual TLS or SignAndEncrypt
  user-token policies.
- *IEC 61850*: SCL configuration files validated against the
  Edition 2.1 schema; IEC 62351 security applied for GOOSE/SV.
- *IEC 60870-5-104*: TLS encapsulation and certificate-based peer
  authentication.
- *DNP3*: Secure Authentication v5 with key-management governance.
- *MQTT*: MQTT 5 with mutual TLS; broker policies record the
  retain and QoS budgets per topic.
- *AMQP / Kafka*: SASL with mutual TLS; topic-level ACLs aligned
  with the federation's IEC 62443 zone model.

Protocol-binding deviations (e.g. unsigned GOOSE, plain-text
DNP3) are documented as deficiencies in the federation's
quality-management dossier and remediated per the operator's
risk register (ISO 31000).

## §7 Namespace Stewardship

The naming authority (§2) maintains the namespace mapping under a
slow-cadence change-control cycle (typically quarterly). Changes
to namespace mappings are particularly sensitive because
downstream consumers cache resolved identifiers; namespace-revision
change-control records publish a deprecation window during which
the prior identifier and the new identifier both resolve.

## §8 Replay-Window Discipline

Replay traffic on production flows masks current real-time events
and can mislead operators. Discipline requires:

- replay windows are explicitly approved by the change-control
  board chair;
- replay traffic is tagged so subscribers can filter it out of
  decision flows that must reflect current state;
- replay completion is verified before the window record is
  marked completed;
- replay artefacts are retained for the period required by the
  triggering investigation or revalidation.

## §9 Cybersecurity (IEC 62443)

Each contributing system is placed in an IEC 62443 zone whose
target Security Level (SL-T) is recorded in the federation's
zone-and-conduit model. Conduits between zones (the integration
broker's flows) carry the security controls required for the
higher of the connected zones' SL-T values.

Federations whose zone model includes the operator's industrial-
security broker as the canonical conduit terminate flows there;
direct cross-zone flows that bypass the broker are not allowed
under this PHASE.

## §10 Cybersecurity for Substation Communications

Federations that include IEC 61850 substation systems apply IEC
62351 security to the substation communications: GOOSE/SV
authentication, role-based access control on MMS, and TLS for
the management channels. The federation records the IEC 62351
profile reference per substation in the system descriptor's
`certificationRefs`.

## §11 Data Stewardship Through Vendor Upgrades

Vendor upgrades to a contributing system trigger automatic re-
validation of semantic mappings whose source or target was the
upgraded system. The integration architect re-validates mappings
through the change-control process (§5) before promoting them
back to production-grade.

Vendor end-of-life triggers a decommissioning workflow (PHASE-1
§11) under change control.

## §12 Quality-Management Dossier

The operator's quality-management dossier records the certified
ISO 9001 / ISO/IEC 27001 / IEC 62443 scope of the federation's
operations, the federation governance charter reference, the
naming authority charter, the change-control board terms of
reference, the namespace mapping reference, and the deprecation
history of contributing systems and protocol bindings. The
dossier is reviewed annually by the operator's quality manager.

## §13 Cross-Border / Multi-Operator Federations

Federations that span jurisdictions or organisations honour each
participating jurisdiction's regulatory regime (NERC CIP for
North American bulk-electric systems, NIS2 for EU operators of
essential services, K-ISMS-P for Korean critical-infrastructure
operators, equivalent regimes elsewhere). The federation records
the applicable regulator references per zone so downstream
consumers see which authority's requirements the federation
honours.

## §14 Programme Wind-Down

A federation that ceases operations transfers its records to a
recognised long-term archive, exports the change-control register
to the operator's records-management system, and notifies
regulators of the cessation. Records subject to indefinite
retention (cybersecurity audit logs, regulator notification
records, incident records) transfer with content-addresses
preserved.

## §15 Mapping Lineage Discipline

Every value the federation publishes downstream is the result of a
chain of semantic mappings (PHASE-1 §5) whose lineage MUST be
traceable end-to-end. Lineage requirements:

- the lineage is recorded as a directed acyclic graph whose nodes
  are mapping identifiers and whose edges record the input-to-
  output relationship;
- the lineage is queryable via the API's lineage endpoint
  (PHASE-2 §6) without invoking the source contributing systems,
  so that lineage queries do not increase load on the source
  systems;
- when a mapping is deprecated, the lineage graph retains the
  deprecated mapping for citation purposes but marks it
  superseded by its successor.

## §16 Synchronisation and Time

All contributing systems synchronise their clocks to the
federation's reference time source (NTPv4 stratum-2 or better
per RFC 5905, or a dedicated PTP grandmaster for federations
whose flows include sub-second-deadline traffic such as IEC
61850 GOOSE). Clock-skew observations against the reference
source are recorded in the broker's observability output and
emit priority-1 events when the skew exceeds the per-flow
latency budget.

## §17 Failure-Mode Discipline

Federations maintain a failure-mode register that records the
known degraded states of the integration fabric: contributing
system unavailable, broker partial-failure, mapping mis-
classification, namespace conflict, replay collision, security-
broker rule rejection. Each entry records the detection signal,
the operator action, the consumer-impact statement, and the
rollback or recovery procedure.

The failure-mode register is reviewed at the same cadence as the
quality-management dossier and is exercised at least annually
through a tabletop incident-response rehearsal whose evidence is
appended to the federation's audit trail.

## §18 Conformance and Auditing

A federation conformant with WIA-infrastructure-integration
publishes its governance charter, its programme registration in
the discovery document, the catalogue of contributing systems,
the catalogue of message flows, the change-control register, and
the cyber-posture summary, and answers an annual self-assessment
that maps each clause of this PHASE to the federation's
implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-infrastructure-integration
- **Last Updated:** 2026-04-28
