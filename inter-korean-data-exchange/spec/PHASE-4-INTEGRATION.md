# WIA-inter-korean-data-exchange PHASE 4 — INTEGRATION Specification

**Standard:** WIA-inter-korean-data-exchange
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an authorised inter-Korean exchange
operator integrates with the systems that surround it: the
Ministry of Unification authorisation system; the Inter-Korean
Exchange and Cooperation Bureau; the Korea Red Cross
administration system; the Korea Customs Service for
humanitarian-aid shipment verification; KOICA programme
management; sanctions-screening providers and the UN Sanctions
Committee notification intake (where exemption requests apply);
the inter-Korean liaison channel; long-term archives of the
Korea National Archives; and citation tools that resolve
published reports back to their evidence packages.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- UN OCHA Common Operational Datasets
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Ministry of Unification Authorisation Integration

The Ministry of Unification operates the authorisation system
that issues per-domain and per-shipment authorisations under
the Inter-Korean Exchange and Cooperation Act. Integration
carries the Ministry's identifier, the per-authorisation
content-address of the issued letter, the issuance date, the
validity expiry, and the suspension events that affect the
authorisation.

## §2 Korea Red Cross Integration

The Korea Red Cross operates the south-side family-reunion
list, the south-side humanitarian-aid distribution to north-
side counterparts (KP Red Cross), and the inter-Korean
liaison-correspondence administration where the Red Cross is
the named conduit. Integration carries the Red Cross's
identifier, the per-reunion-round operating responsibilities,
and the per-aid-shipment beneficiary classification chain.

## §3 KOICA Programme Integration

KOICA (Korea International Cooperation Agency) consumes
humanitarian-aid manifests for programmes that flow through
KOICA's bilateral or multilateral funding channels.
Integration carries KOICA's identifier, the per-programme
funding reference, and the per-shipment reporting cadence
that KOICA requires.

## §4 Korea Customs Service Integration

Humanitarian-aid shipments that cross south-side custom
boundaries follow the Korea Customs Service's customs-
declaration procedures. Integration carries the Customs
Service's identifier, the per-shipment customs-declaration
reference, and the operator's reconciliation workflow when
customs-declaration data and aid-manifest data disagree
(typically resolved through the Customs Service's amendment
process before shipment release).

## §5 Sanctions Screening Provider Integration

Sanctions screening providers (Refinitiv, Dow Jones, in-house
ministry feeds) emit periodic refreshes of the UN Security
Council Sanctions Committee's consolidated list against the
DPRK and the south-side implementing rules. The operator's
sanctions sweep (PHASE-3 §2) consumes the refreshes through
this integration; sweep events update each affected exchange
artefact's `sanctionsScreenAt` timestamp.

## §6 Sanctions Committee Exemption Intake Integration

For shipments that require a UN Sanctions Committee exemption
(humanitarian aid that would otherwise be blocked by
sanctions), the operator submits an exemption request through
the Committee's notification intake. Integration carries the
Committee's intake reference, the operator's exemption
request artefact, and the Committee's eventual ruling.

## §7 Inter-Korean Liaison Channel Integration

The inter-Korean liaison channel is the primary correspondence
conduit. When the Joint Liaison Office at Kaesong is
operating, integration is direct; during suspensions, the
operator's correspondence flows through the Panmunjom
exchange route, the ministry-direct courier route, or
third-country conduits. The integration record carries the
active channel and the channel-fallback history.

## §8 Korea National Archives Integration

Records that have completed their operational lifecycle
transfer to the Korea National Archives under the South Korean
Government Records Management Act. Integration carries the
Archives' identifier, the per-record-class deposit cadence,
and the content-address preservation contract so that
downstream historians can resolve archived inter-Korean
exchange records by their original content-addresses.

## §9 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  authorisations/              — Ministry of Unification
                                  authorisations and amendments
  identity-tokens/             — token records (no PII)
  family-reunions/             — reunion records and outcomes
  aid-manifests/               — manifest records and shipment
                                  history
  liaison-correspondences/     — correspondence metadata (body
                                  references only; bodies held
                                  in the operator's secure
                                  document store)
  joint-venture-inventories/   — inventory snapshots
  sanctions-screens/           — sanctions-sweep summaries
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed by
the operator's HTTP-message-signature key (RFC 9421) and
counter-signed by the Ministry of Unification's records
custodian when the package supports a regulatory or archival
submission.

## §10 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:inter-korean-data-exchange:evidence-mismatch`.

## §11 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-inter-korean-data-exchange` that links to
the API root, the Ministry of Unification authorisations
summary, the published quality dossier, the suspension status
per exchange domain, and the catalogue of supported reunion
rounds and aid shipments.

## §12 Public Catalogue and Aggregator Feeds

Operators that publish a public catalogue of completed reunion
rounds and humanitarian-aid shipments emit an Atom or JSON
Feed listing the records with their evidence-package manifest
digests, the round identifier or shipment reference, the
beneficiary classification (for aid), and the resolved date.
The feed never carries PII; reunion-round entries reference
opaque round identifiers, and aid-shipment entries reference
opaque manifest identifiers.

## §13 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (Ministry of
Unification authorisation status, ISO/IEC 27001 certification,
Sphere Standards adherence) to consumers of W3C Verifiable
Credentials MAY re-issue the attestations as Verifiable
Credentials under the Data Model 2.0 specification. Re-
issuance is optional; the canonical record remains the JSON
evidence-package manifest.

## §14 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with
prior-minor clients. Major revisions go through a deprecation
window of at least 18 calendar months so that Ministry of
Unification, Korea Red Cross, KOICA, Customs Service, and
NGO partner integrations have time to migrate; the longer
window reflects the slower pace at which inter-government
integrations adapt.

## §15 Cross-Standard Linkage

Operators that consume adjacent WIA standards (WIA-disaster-
relief for emergency-aid coordination, WIA-cultural-heritage
for cross-DMZ heritage repatriation records) emit cross-
standard linkage records that name the consuming standard
and the version under which the linkage is claimed.

## §16 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support. Subscribers that disconnect
during long aid-shipment windows resume from the last seen
event identifier without losing visibility of priority-1
sanctions-screening or shipment-status events.

## §17 Long-Term Archive Continuity

The Korea National Archives integration of §8 is the operator's
long-term archive of record. Operators that designate
additional archives (academic institutions for scholarly-
exchange records, museum custodians for cultural-exchange
records) record the secondary archive's identifier and the
deposit policy.

## §18 Suspension-Notification Integration

Ministry of Unification suspension orders flow through the
Inter-Korean Exchange and Cooperation Bureau's notification
endpoint. The operator's API consumes the notification, freezes
in-flight artefacts under the affected domain, and publishes
the suspension status through the discovery document. Lift
events flow through the same channel; the operator restarts
the affected domain under the lift's fresh authorisations.

## §19 Cultural-Heritage Custodian Integration

Heritage repatriation programmes (south-side return of north-
side artefacts located in south-side custodianship, or
joint exhibitions of artefacts whose custody is split across
the DMZ) integrate with cultural-heritage custodians: the
National Museum of Korea, the National Folk Museum, the
Cultural Heritage Administration, and academic-institution
custodians. The integration carries the custodian's
identifier, the per-artefact provenance chain, and the
return-protocol agreement that governs the repatriation.

## §20 Funder and Donor Integration

Humanitarian-aid programmes funded by external donors
(international foundations, faith-based donor agencies,
diaspora donor organisations) integrate with the donor's
reporting system. The integration carries the donor's
identifier, the per-grant funding reference, and the donor's
reporting cadence; donors do not consume sanctions-screening
detail directly but receive the operator's attestation that
sanctions discipline (PHASE-3 §2) has been applied to every
shipment funded under the donor's grant.

## §21 Press and Public Disclosure Integration

Inter-Korean exchange events of public-interest significance
(reunion rounds, large humanitarian-aid shipments, cultural
exchanges) flow through the operator's press-disclosure
workflow. The integration carries the operator's press
contact, the per-event press-release artefact, and the public
disclosure timestamp. PII rules (PHASE-3 §9) apply to any
press release; reunion-round press content carries opaque
participant tokens and aggregate counts only.

## §22 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with the Ministry of Unification, the Korea Red Cross or KOICA
or another authorised humanitarian counterpart, the Korea
Customs Service (for the humanitarian-aid domain), at least
one sanctions-screening provider, and the Korea National
Archives, and has published at least one externally citable
evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-inter-korean-data-exchange
- **Last Updated:** 2026-04-28
