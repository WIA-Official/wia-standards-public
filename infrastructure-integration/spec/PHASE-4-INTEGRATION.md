# WIA-infrastructure-integration PHASE 4 — INTEGRATION Specification

**Standard:** WIA-infrastructure-integration
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited infrastructure-integration
federation integrates with the systems that surround it: contributing
systems' vendor-specific management consoles; integration brokers
and ESB platforms operating in production; the operator's
industrial-security broker and IEC 62443 zone-and-conduit gateways;
historian archives; long-term archives; the operator's identity-
provider for change-control board members and integration architects;
regulators that audit critical-infrastructure deployments; vendor
support portals for incident escalation; and citation tools that
resolve published flow descriptions to evidence packages. It also
defines the evidence-package format that bundles a federation's
complete record set for external audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- IEC 62443 (industrial cybersecurity)
- IEC 61850 (substation automation)
- IEC 62541 (OPC UA)
- IEC 61968 / 61970 (CIM)
- W3C Verifiable Credentials Data Model 2.0 (optional)
- buildingSMART IFC4.3 (infrastructure asset linkage)

---

## §1 Vendor Management-Console Integration

Contributing systems are administered through vendor-specific
management consoles that hold the in-system configuration
artefacts (OPC UA NodeSet exports, IEC 61850 SCL files, BACnet
device descriptions, Kafka topic configurations). The integration
is read-only from the federation side: the federation consumes
vendor-published configuration exports to populate the protocol-
binding records (PHASE-1 §4) and the semantic-mapping source/
target schemas. Bidirectional configuration push is *not*
defined under this PHASE because most vendor consoles do not
support it safely.

## §2 Integration-Broker / ESB Platform Integration

The federation's integration broker (or ESB) is the runtime
that executes the message flows recorded in PHASE-1 §7. The
broker integration carries the deployed flow definitions, the
runtime quality-of-service metrics, the per-flow error counters,
and the active backpressure indicators. Brokers emit events on
the federation's event stream (PHASE-2 §16) when their runtime
QoS observations deviate from the contracted QoS budgets.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  federation.json              — federation record
  systems/                     — contributing-system descriptors
  protocol-bindings/           — protocol-binding records
  semantic-mappings/           — semantic-mapping records and
                                 lineage
  namespace-mapping/           — namespace mapping snapshot
  message-flows/               — message-flow catalogue
  audit-trails/                — audit-trail records
  change-controls/             — change-control register
  cyber-posture/               — IEC 62443 zone-and-conduit model
  archive/                     — long-term archive deposit
                                 references
```

The package is content-addressable; the manifest is signed by
the federation operator and counter-signed by the change-control
board chair when the package supports a regulatory submission.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the manifest,
and reject the package on mismatch with type
`urn:wia:infrastructure-integration:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant federation exposes a discovery document at
`/.well-known/wia-infrastructure-integration` that links to the
API root, the governance charter content-address, the
certification register, and the catalogue of contributing
systems.

## §6 Industrial-Security Broker Integration

The operator's industrial-security broker terminates cross-zone
conduits between IEC 62443 zones. Federation brokers integrate
with the security broker for cross-zone traffic; the integration
record carries the security broker's reference, the conduit
identifier, and the firewall and IDS rules that govern the
conduit.

## §7 Identity-Provider Integration

Change-control board members and integration architects
authenticate via the operator's identity provider; the
integration carries the IdP's metadata reference, the role
mapping from IdP groups to federation roles (board member,
architect, reviewer), and the role-revocation propagation
deadline.

## §8 Historian Archive Integration

Historian archives consume PHASE-1 §7 message flows and emit
long-term retained values for trend analysis. The integration
records the historian's identifier, the per-flow retention
period, the storage tier (hot / warm / cold), and the retention
deletion policy when the retention period elapses.

## §9 Long-Term Archive Integration

Federations designate a long-term archive that holds the
federation's records beyond programme wind-down. Quarterly
deposits round-trip content-addresses; on wind-down, remaining
records transfer to the archive with content-addresses
preserved. Recognised archives include national archives,
professional-society archives, and trustworthy digital
repositories accredited under ISO 16363 or equivalent.

## §10 Regulator Notification Integration

Regulators of critical-infrastructure operations consume incident
notifications, change-control records that affect the federation's
boundary with the regulated zone, and annual self-assessment
submissions. The integration record carries the regulator's
identifier, the notification format the regulator requires, and
the maximum notification latency.

## §11 Vendor Support Portal Integration

Vendor support portals consume incident escalation requests when
a contributing system's behaviour deviates from its published
conformance class. The integration carries the vendor's portal
URI, the support-contract reference, and the per-system
escalation paths.

## §12 Worked Example: Citation Resolution for a Federation Flow

A reader encounters a peer-reviewed publication that cites the
federation's message flow as the canonical source for a published
analytics result. The reader's tool resolves the citation by:

1. Parsing the citation to extract the federation ID and manifest
   digest.
2. Fetching the discovery document for the federation.
3. Resolving the manifest URL and verifying the manifest
   signatures.
4. Recomputing the manifest digest and comparing it to the
   pinned digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests;
   surfacing the resolved evidence (flow record, semantic
   mappings, namespace mapping snapshot, audit trails) to the
   reader.

A conformant tool completes this flow without further input.

## §13 Cross-Standard Linkage

Federations that consume adjacent WIA standards (infrastructure
for asset records, infrastructure-monitoring for SCADA telemetry,
intelligent-transportation for traffic operations, electric-grid
for transmission, water-supply for water utilities) emit cross-
standard linkage records. Linkage records bind a federation
identifier to the adjacent standard's identifier so that
auditors can navigate from a flow's evidence package to the
asset records that the flow's payloads describe.

## §14 Reader Tooling

Federations MAY publish supplementary reader hints (visual
topology maps, mapping lineage diagrams, audit-trail timelines)
alongside the canonical evidence package. Reader tools are
non-normative and remain under federation control.

## §15 Verifiable-Credential Re-Issuance (optional)

Federations that wish to expose attestations (ISO/IEC 27001
certification, IEC 62443 SL-T attestations, integration architect
qualifications) to consumers of W3C Verifiable Credentials MAY
re-issue the attestations as Verifiable Credentials under the
Data Model 2.0 specification. Re-issuance is optional; the
canonical record remains the JSON evidence-package manifest.

## §16 Streaming Heartbeat

SSE subscribers (PHASE-2 §16) receive a heartbeat every 30
seconds; replays support `Last-Event-ID` headers (W3C
EventSource semantics). Subscribers that disconnect resume from
the last seen event identifier without losing visibility of
priority-1 incident events.

## §17 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window
of at least one full ISO/IEC 27001 surveillance cycle.

## §18 Vendor-Onboarding Workflow

New contributing systems are onboarded under a documented
workflow: vendor-published conformance evidence is reviewed,
the system's IEC 62443 zone is assigned, the protocol bindings
are registered, the initial semantic mappings are drafted and
counter-validated, and the system is staged in a non-production
broker before promotion to production. The workflow's evidence
is bound to the system descriptor in the federation's
quality-management dossier.

## §19 Asset-Spine Linkage

Federations that operate alongside an asset-management programme
(WIA-infrastructure) emit linkage records that bind asset
identifiers to the contributing systems' local identifiers via
the namespace mapping (PHASE-1 §6). Linkage records make it
possible to resolve an asset's WIA-infrastructure record from a
flow event, and conversely to find the flows that carry data
about an asset.

## §20 Multi-Vendor Test-Bed Integration

Federations that operate in safety-critical domains (substation
automation, water-utility SCADA, transit operations) maintain a
multi-vendor test bed that mirrors the production zone-and-conduit
topology and replays representative traffic for change-control
verification (PHASE-3 §5). The test-bed integration carries the
test-bed's identifier, the version of the broker firmware
deployed, the per-flow simulation profiles, and the scheduled
rehearsal cadence.

Change-control records (PHASE-1 §10) cite the test-bed
verification result before the change is approved for production
rollout; submissions whose test-bed verification reference is
missing return `urn:wia:infrastructure-integration:test-bed-
verification-required`.

## §21 Tabletop-Rehearsal Integration

The annual incident-response rehearsal mandated under PHASE-3
§17 is run jointly with the operator's CSIRT, the contributing-
system vendors, and the regulator-facing representative. The
integration carries the rehearsal scenario register, the dates
of upcoming rehearsals, and the per-rehearsal outcome reports
referenced into the federation's audit trail.

## §22 Mapping-Lineage Visualisation

Federations MAY publish supplementary lineage visualisations
that render the directed acyclic graph of mappings (PHASE-3 §15)
as an interactive diagram so that integration architects and
auditors can navigate the lineage without authoring SPARQL
queries against the federation's metadata triple store. The
visualisations are non-normative and remain under federation
control; the canonical lineage record is the JSON returned by
the lineage endpoint.

## §23 Conformance and Sunset

A federation conformant with PHASE-4 has integrated successfully
with at least one vendor management console, at least one
integration broker, the operator's industrial-security broker,
the operator's identity provider, at least one historian
archive, the relevant regulator's notification intake, and at
least one long-term archive, and has published at least one
externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-infrastructure-integration
- **Last Updated:** 2026-04-28
