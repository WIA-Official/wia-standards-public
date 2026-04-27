# WIA-industrial-iot PHASE 4 — INTEGRATION Specification

**Standard:** WIA-industrial-iot
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited industrial-IoT programme
integrates with the systems that surround it: OPC UA servers and
clients; SCADA and DCS front-ends; MES and APM platforms; ERP
systems; CMMS and asset-management systems; cybersecurity
operations centres; long-term archives; and regulators that
inspect deployments under the relevant industrial-cybersecurity
or product-liability regimes.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27019:2024 (IS controls for energy utilities)
- ISO 8601 (date and time)
- IEC 62264 (ISA-95)
- IEC 62443 (industrial automation security)
- OPC Unified Architecture (UA)
- W3C Verifiable Credentials Data Model 2.0 (optional re-issuance
  of operator attestations)

---

## §1 OPC UA Integration

OPC UA servers running on the shop floor expose telemetry through
companion specifications appropriate to the equipment class
(Robotics, Machinery, Pumps & Vacuum, etc.). The WIA programme
integrates as an OPC UA client through a dedicated companion-
spec adapter that maps OPC UA AddressSpace nodes to PHASE-1 §3
asset records and PHASE-1 §4 telemetry samples.

Adapters are owned by the operating programme and are exercised
during firmware updates of OPC UA servers; submissions whose
companion-spec version cannot be resolved against the registered
adapters return `urn:wia:industrial-iot:companion-spec-unresolved`.

## §2 SCADA and DCS Front-End Integration

SCADA / DCS front-ends consume telemetry and emit operator
actions (alarm acknowledgement, control-mode changes, setpoint
adjustments). The integration is bidirectional and is mediated
by the operator's industrial-security broker; the WIA API never
authenticates a field operator directly.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  site.json                    — site record
  assets/                      — asset records and capability
                                  profile references
  telemetry/                   — telemetry summaries and historian
                                  references (raw streams remain
                                  in the historian)
  control-loops/               — loop records and tuning history
  alarm-events/                — alarm and event history
  maintenance/                 — maintenance records and ISO 14224
                                  failure codes
  cyber-posture/               — IEC 62443 zone/conduit definitions
  production-orders/           — production orders and trace
                                  references
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed by
the operating programme and counter-signed by the certifying
body when the package is consumed for IEC 62443 / ISO/IEC 27019
audit purposes.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:industrial-iot:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-industrial-iot` that links to the API root,
the public operator accreditation references, the published
quality dossier, the IEC 62443 architecture summary, and the
catalogue of sites the programme operates.

## §6 MES and APM Integration

MES platforms consume production-order records and emit
execution updates. APM platforms consume telemetry and
maintenance records and emit reliability KPIs. Both integrations
are mutually authenticated and scoped to the consumer's
authorisation set.

## §7 ERP Integration

ERP systems consume production-order completion events to
trigger downstream finished-goods inventory updates and customer-
order release. The integration is one-way (shop floor → ERP for
production completion; ERP → shop floor for new order release)
and is mediated by an integration broker that translates between
the WIA API's record shapes and the ERP's native message format.

## §8 CMMS Integration

The Computerised Maintenance Management System holds the work-
order register and the maintenance scheduler. The CMMS
integration consumes maintenance records and emits new work
orders triggered by predictive-condition-based or preventive-
scheduled cadence.

## §9 Cybersecurity Operations Centre Integration

The operating organisation's CSIRT consumes priority-1 and
priority-2 security-category alarm events through the streaming
subscription (PHASE-2 §13). The CSIRT's response actions are
appended to the event record so that post-incident review can
reconstruct the response timeline.

## §10 Long-Term Archive Integration

Programmes designate a long-term archive that holds historian
exports, alarm history, maintenance records, and evidence
packages beyond programme wind-down. Quarterly deposits round-
trip content-addresses; on wind-down, remaining records transfer
to the archive with content-addresses preserved.

## §11 Regulator and Certifying Body Access

Regulators (energy regulator, occupational-safety inspectorate,
product-liability authority where applicable) and certifying
bodies (IEC 62443 conformance assessor, ISO 9001 / ISO/IEC 27001
auditor) access the API via dedicated client certificates issued
by the certifying body. Access scopes for these clients include
the full record set; consumer-facing scopes (MES, APM, ERP) are
narrower.

## §12 Worked Example: Citation Resolution for a Reliability KPI

A reader encounters a peer-reviewed publication or industry case
study that cites an MTBF KPI for a class of equipment. The
reader's tool resolves the citation by:

1. Parsing the citation to extract the asset class, the period,
   and the manifest digest.
2. Fetching the discovery document for the operating programme.
3. Resolving the manifest URL and verifying the manifest
   signatures.
4. Recomputing the manifest digest and comparing it to the
   pinned digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests;
   surfacing the resolved evidence (asset records, maintenance
   records, ISO 14224 failure codes, computed KPI) to the reader.

A conformant tool completes this flow without further input.

## §13 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (predictive-
maintenance, manufacturing-automation, network-protocol for
field-bus modernisation) emit cross-standard linkage records
that name the consuming standard and the version under which
the linkage is claimed.

## §14 Reader Tooling

Programmes MAY publish supplementary reader hints (visual asset
hierarchy maps, alarm-rate dashboards, KPI rollup charts)
alongside the canonical evidence package. Reader tools are non-
normative.

## §15 Public Catalogue

Programmes that publish a public catalogue of operating sites
emit an Atom or JSON Feed listing sites with their evidence-
package manifest digests, the security level, and the asset-
class summary. The feed does not carry telemetry detail; it is
intended for industry transparency and supply-chain analysis.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window
of at least one full ISO 9001 surveillance cycle.

## §17 Migration from Pre-Standard Records

Programmes that operated before WIA-industrial-iot reached
version 1.0 MAY migrate historical sites by emitting synthetic
site records with a `legacyImport` flag. Synthetic sites are
accepted by the public catalogue but are not eligible for
evidence-package generation without contemporaneous re-validation
of the IEC 62443 zone architecture.

## §18 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose attestations (IEC 62443
conformance, ISO/IEC 27001 conformance, alarm-rate-budget
compliance) to consumers of W3C Verifiable Credentials MAY re-
issue the attestations as Verifiable Credentials under the Data
Model 2.0 specification. Re-issuance is optional; the canonical
record remains the JSON evidence-package manifest.

## §19 Predictive-Maintenance Service Integration

Predictive-maintenance services consume telemetry summaries
(vibration spectra, lubricant condition signatures, motor-current
signatures) and emit failure-imminent alerts that drive
condition-based maintenance work orders. The integration is
mediated by APM (PHASE-4 §6) but predictive-maintenance vendors
may also subscribe directly via the streaming endpoint with
operator-issued client certificates scoped to the targeted asset
classes.

## §20 ISO 14224 Reliability Data Submission

Operators that participate in industry-wide reliability databases
(per ISO 14224 collection conventions) submit anonymised
reliability records on the database's required cadence. The
integration carries the database's identifier, the operator's
membership status, and the per-asset-class anonymisation rules
that the operator applies before submission.

## §21 Sustainability Reporting Platform Integration

Sustainability-disclosure platforms (CSRD reporting tools, GRI
reporting tools, SASB-aligned ESG suites) consume the
sustainability rollup endpoint (PHASE-2 §18) and emit completed
disclosure documents. The integration carries the platform's
identifier, the disclosure regime in scope, and the per-period
aggregation methodology that the operator has declared.

## §22 Streaming Heartbeat

SSE subscribers (PHASE-2 §13) receive a heartbeat every 30
seconds; replays support `Last-Event-ID` headers (W3C
EventSource semantics). Subscribers that disconnect during
long telemetry windows resume from the last seen event
identifier without losing visibility of priority-1 alarm events.

## §23 Reader Tooling for Reliability and Operations

Reliability engineers, plant managers, and certifying-body
inspectors benefit from visualisation tools that surface MTBF /
MTTR rollups, alarm-rate dashboards, and configuration-snapshot
diff views. Programmes MAY publish reader tools alongside the
canonical evidence package; the tools are non-normative.

## §24 Cross-Site Federation Adapter

Multi-site operators that share APM, MES, or historian services
across sites integrate through a federation adapter that
translates between per-site WIA records and the federated
service's combined view. The adapter honours each site's
authorisation scope so that federated queries respect the source
site's IEC 62443 zone classifications.

## §25 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with at least one OPC UA server, at least one SCADA / DCS front-
end, at least one MES, at least one APM, at least one ERP, the
operator's CMMS, the operator's CSIRT, and at least one long-
term archive, and has published at least one externally citable
evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-industrial-iot
- **Last Updated:** 2026-04-27
